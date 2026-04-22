#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <openvino/openvino.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace fs = std::filesystem;

namespace {
constexpr int kChannels = 3;
constexpr int kExpectedAttributes = 5;

inline float sigmoid(float x) {
  return 1.0f / (1.0f + std::exp(-x));
}

struct LetterboxResult {
  cv::Mat image;
  float scale = 1.0f;
  int pad_x = 0;
  int pad_y = 0;
};

enum class DetectionKind : uint8_t {
  Victim = 0,
  QRCode = 1
};

struct Detection {
  cv::Rect box;
  float score = 0.0f;

  bool has_position = false;
  cv::Point3f position;
  int support_points = 0;

  DetectionKind kind = DetectionKind::Victim;
  std::string label = "victim";

  bool use_center_window_average = false;
  cv::Point2f image_center = cv::Point2f(-1.0f, -1.0f);
  int center_window_size = 9;

  int qr_reference_index = -1;
};

struct StageTimings {
  double preprocess_ms = 0.0;
  double inference_ms = 0.0;
  double decode_ms = 0.0;
  double qr_detect_ms = 0.0;
  double point_fit_ms = 0.0;
  double draw_ms = 0.0;
  double publish_ms = 0.0;
};

struct PointXYZ {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct DetectionPoint {
  cv::Point3f position;
  float confidence = 0.0f;
};

struct CloudFieldOffsets {
  int x = -1;
  int y = -1;
  int z = -1;
  bool valid() const { return x >= 0 && y >= 0 && z >= 0; }
};

struct QrReferenceTemplate {
  std::string name;
  cv::Mat binary;
};

inline cv::Point3f mapPointToCameraOutput(const cv::Point3f& point) {
  // Raw fitted data: x=left, y=up, z=front.
  // Requested publish convention:
  // right=-x, up=y, front=-z
  return cv::Point3f(-point.x, point.y, -point.z);
}
}  // namespace

class VisualCalibrationNode {
 public:
  using Clock = std::chrono::steady_clock;

  VisualCalibrationNode()
      : nh_(),
        pnh_("~"),
        image_transport_(nh_),
        core_(),
        qr_detector_() {
    image_topic_ = pnh_.param<std::string>("image_topic", "/camera/color/image_raw");
    cloud_topic_ = pnh_.param<std::string>("cloud_topic", "/camera/depth_registered/points");
    model_path_ = pnh_.param<std::string>("model_path", defaultModelPath());

    visualization_topic_ =
        pnh_.param<std::string>("visualization_topic", "/visual_calibration/visualization");

    victim_cloud_topic_ =
        pnh_.param<std::string>("detection_cloud_topic", "/visual_calibration/detection_points");
    qr_cloud_topic_ =
        pnh_.param<std::string>("qr_detection_cloud_topic", "/visual_calibration/qr_detection_points");

    detection_cloud_frame_ =
        pnh_.param<std::string>("detection_cloud_frame", "camera_depth_frame");

    inference_device_ = pnh_.param<std::string>("inference_device", "AUTO:GPU,CPU");
    performance_mode_ = pnh_.param<std::string>("performance_mode", "THROUGHPUT");

    rotate_180_ = pnh_.param<bool>("rotate_180", true);
    conf_threshold_ = pnh_.param<double>("conf_threshold", 0.65);
    nms_threshold_ = pnh_.param<double>("nms_threshold", 0.45);
    input_size_ = pnh_.param<int>("input_size", 640);

    enable_profiling_log_ = pnh_.param<bool>("enable_profiling_log", true);
    profiling_log_interval_ = pnh_.param<int>("profiling_log_interval", 60);

    point_sample_step_ = std::max(1, pnh_.param<int>("point_sample_step", 2));
    min_valid_points_ = std::max(4, pnh_.param<int>("min_valid_points", 12));
    depth_keep_ratio_ = pnh_.param<double>("depth_keep_ratio", 0.30);
    depth_keep_ratio_ = std::clamp(depth_keep_ratio_, 0.05, 1.0);

    victim_small_box_area_thresh_ =
        std::max(0, pnh_.param<int>("victim_small_box_area_thresh", 80 * 80));
    victim_center_window_size_ =
        std::max(3, pnh_.param<int>("victim_center_window_size", 9));
    if ((victim_center_window_size_ % 2) == 0) ++victim_center_window_size_;
    victim_center_min_valid_points_ =
        std::max(1, pnh_.param<int>("victim_center_min_valid_points", 8));

    victim_max_bbox_area_percent_ =
        std::clamp(pnh_.param<double>("victim_max_bbox_area_percent", 35.0), 0.0, 100.0);

    max_candidates_pre_nms_ =
        std::max(0, pnh_.param<int>("max_candidates_pre_nms", 200));

    enable_qr_detection_ = pnh_.param<bool>("enable_qr_detection", true);
    qr_center_window_size_ = std::max(3, pnh_.param<int>("qr_center_window_size", 9));
    if ((qr_center_window_size_ % 2) == 0) ++qr_center_window_size_;
    qr_min_valid_points_ = std::max(1, pnh_.param<int>("qr_min_valid_points", 8));
    qr_min_box_size_ = std::max(4, pnh_.param<int>("qr_min_box_size", 20));

    qr_template_size_ = std::max(64, pnh_.param<int>("qr_template_size", 256));
    qr_enable_reference_filter_ = pnh_.param<bool>("qr_enable_reference_filter", true);
    qr_reference_match_threshold_ =
        pnh_.param<double>("qr_reference_match_threshold", 0.78);
    qr_reference_dir_ =
        pnh_.param<std::string>("qr_reference_dir", defaultQrReferenceDir());

    qr_detect_interval_ = std::max(1, pnh_.param<int>("qr_detect_interval", 5));
    qr_detect_max_side_ = std::max(160, pnh_.param<int>("qr_detect_max_side", 640));
    qr_reuse_cached_result_ = pnh_.param<bool>("qr_reuse_cached_result", true);

    publish_tf_ = pnh_.param<bool>("publish_tf", true);
    tf_parent_frame_ = pnh_.param<std::string>("tf_parent_frame", detection_cloud_frame_);
    victim_tf_prefix_ = pnh_.param<std::string>("victim_tf_prefix", "victim");
    qr_tf_prefix_ = pnh_.param<std::string>("qr_tf_prefix", "qrcode");

    victim_consecutive_frames_required_ =
        std::max(1, pnh_.param<int>("victim_consecutive_frames_required", 3));
    qr_consecutive_frames_required_ =
        std::max(1, pnh_.param<int>("qr_consecutive_frames_required", 3));

    victim_publish_conf_threshold_ =
        std::clamp(pnh_.param<double>("victim_publish_conf_threshold", conf_threshold_), 0.0, 1.0);
    {
      const double default_qr_pub_conf =
          qr_enable_reference_filter_ ? qr_reference_match_threshold_ : 0.65;
      qr_publish_conf_threshold_ =
          std::clamp(pnh_.param<double>("qr_publish_conf_threshold", default_qr_pub_conf), 0.0, 1.0);
    }

    if (!fs::exists(model_path_)) {
      ROS_FATAL_STREAM("ONNX model not found: " << model_path_);
      throw std::runtime_error("model path does not exist");
    }

    if (input_size_ <= 0) {
      throw std::runtime_error("input_size must be > 0");
    }

    loadModel();
    loadQrReferenceTemplates();

    image_sub_ = image_transport_.subscribe(
        image_topic_, 1, &VisualCalibrationNode::imageCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &VisualCalibrationNode::cloudCallback, this);

    vis_pub_ = image_transport_.advertise(visualization_topic_, 1);
    victim_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(victim_cloud_topic_, 5, true);
    qr_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(qr_cloud_topic_, 5, true);

    input_tensor_values_.resize(
        static_cast<size_t>(kChannels * input_size_ * input_size_));

    worker_thread_ = std::thread(&VisualCalibrationNode::workerLoop, this);

    ROS_INFO_STREAM("visual_calibration listening on: " << image_topic_);
    ROS_INFO_STREAM("cloud topic: " << cloud_topic_);
    ROS_INFO_STREAM("model path: " << model_path_);
    ROS_INFO_STREAM("visualization topic: " << visualization_topic_);
    ROS_INFO_STREAM("victim cloud topic: " << victim_cloud_topic_);
    ROS_INFO_STREAM("qr cloud topic: " << qr_cloud_topic_);
    ROS_INFO_STREAM("detection cloud frame: " << detection_cloud_frame_);
    ROS_INFO_STREAM("OpenVINO device: " << inference_device_);
    ROS_INFO_STREAM("performance mode: " << performance_mode_);
    ROS_INFO_STREAM("point_sample_step: " << point_sample_step_
                    << ", min_valid_points: " << min_valid_points_
                    << ", depth_keep_ratio: " << depth_keep_ratio_);
    ROS_INFO_STREAM("victim_small_box_area_thresh: " << victim_small_box_area_thresh_
                    << ", victim_center_window_size: " << victim_center_window_size_
                    << ", victim_center_min_valid_points: " << victim_center_min_valid_points_);
    ROS_INFO_STREAM("victim_max_bbox_area_percent: "
                    << victim_max_bbox_area_percent_ << "%");
    ROS_INFO_STREAM("max_candidates_pre_nms: " << max_candidates_pre_nms_);
    ROS_INFO_STREAM("QR enabled: " << (enable_qr_detection_ ? "yes" : "no")
                    << ", center window: " << qr_center_window_size_ << "x"
                    << qr_center_window_size_
                    << ", qr_min_valid_points: " << qr_min_valid_points_
                    << ", qr_reference_filter: " << (qr_enable_reference_filter_ ? "on" : "off")
                    << ", qr_reference_dir: " << qr_reference_dir_
                    << ", loaded_refs: " << qr_references_.size()
                    << ", qr_detect_interval: " << qr_detect_interval_
                    << ", qr_detect_max_side: " << qr_detect_max_side_
                    << ", qr_reuse_cached_result: " << (qr_reuse_cached_result_ ? "yes" : "no"));
    ROS_INFO_STREAM("TF publish: " << (publish_tf_ ? "yes" : "no")
                    << ", tf_parent_frame: " << tf_parent_frame_
                    << ", victim prefix: " << victim_tf_prefix_
                    << ", qr prefix: " << qr_tf_prefix_);
    ROS_INFO_STREAM("consecutive publish gate: victim="
                    << victim_consecutive_frames_required_
                    << " frame(s), qr=" << qr_consecutive_frames_required_ << " frame(s)");
    ROS_INFO_STREAM("publish confidence gate: victim>="
                    << victim_publish_conf_threshold_
                    << ", qr>=" << qr_publish_conf_threshold_);
    ROS_INFO_STREAM("profiling: " << (enable_profiling_log_ ? "enabled" : "disabled")
                    << ", interval=" << profiling_log_interval_);
  }

  ~VisualCalibrationNode() {
    running_.store(false);
    frame_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

 private:
  void loadModel() {
    try {
      const auto available_devices = core_.get_available_devices();
      std::string devices;
      for (size_t i = 0; i < available_devices.size(); ++i) {
        if (i > 0) devices += ", ";
        devices += available_devices[i];
      }
      ROS_INFO_STREAM("OpenVINO available devices: " << devices);
    } catch (...) {
      ROS_WARN("Failed to query OpenVINO available devices");
    }

    model_ = core_.read_model(model_path_);

    ov::AnyMap config;
    if (performance_mode_ == "LATENCY") {
      config[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::LATENCY;
    } else {
      config[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::THROUGHPUT;
    }

    compiled_model_ = core_.compile_model(model_, inference_device_, config);
    infer_request_ = compiled_model_.create_infer_request();
    input_port_ = compiled_model_.input();
    output_port_ = compiled_model_.output();
  }

  std::string defaultModelPath() const {
    const auto package_path = fs::path(ros::package::getPath("visual_calibration"));
    if (package_path.empty()) {
      return "src/visual_calibration/models/yolo.onnx";
    }
    return (package_path / "models" / "yolo.onnx").string();
  }

  std::string defaultQrReferenceDir() const {
    const auto package_path = fs::path(ros::package::getPath("visual_calibration"));
    if (package_path.empty()) {
      return "src/visual_calibration/models/qr_refs";
    }
    return (package_path / "models" / "qr_refs").string();
  }

  void loadQrReferenceTemplates() {
    qr_references_.clear();

    std::vector<std::string> reference_paths;
    if (!pnh_.getParam("qr_reference_paths", reference_paths)) {
      reference_paths.clear();
    }

    if (reference_paths.empty() && !qr_reference_dir_.empty() && fs::exists(qr_reference_dir_)) {
      std::vector<fs::path> files;
      for (const auto& entry : fs::directory_iterator(qr_reference_dir_)) {
        if (!entry.is_regular_file()) continue;
        const std::string ext = entry.path().extension().string();
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" ||
            ext == ".bmp" || ext == ".webp") {
          files.push_back(entry.path());
        }
      }
      std::sort(files.begin(), files.end());
      for (const auto& p : files) {
        reference_paths.push_back(p.string());
      }
    }

    for (const auto& path_str : reference_paths) {
      if (!fs::exists(path_str)) {
        ROS_WARN_STREAM("QR reference image not found: " << path_str);
        continue;
      }

      cv::Mat img = cv::imread(path_str, cv::IMREAD_COLOR);
      if (img.empty()) {
        ROS_WARN_STREAM("Failed to read QR reference image: " << path_str);
        continue;
      }

      cv::Mat binary;
      if (!normalizeQrReferenceImage(img, binary)) {
        ROS_WARN_STREAM("Failed to normalize QR reference image: " << path_str);
        continue;
      }

      QrReferenceTemplate ref;
      ref.name = fs::path(path_str).stem().string();
      ref.binary = binary;
      qr_references_.push_back(ref);
    }

    if (qr_enable_reference_filter_ && qr_references_.empty()) {
      ROS_WARN_STREAM("QR reference filter is enabled but no reference templates were loaded. "
                      "Node will accept generic QR detections.");
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_image_msg_ = msg;
      has_new_frame_ = true;
    }
    frame_cv_.notify_one();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = msg;
    cloud_offsets_ = extractCloudFieldOffsets(*msg);
  }

  void workerLoop() {
    while (ros::ok() && running_.load()) {
      sensor_msgs::ImageConstPtr image_msg;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait(lock, [this]() {
          return !running_.load() || has_new_frame_;
        });
        if (!running_.load()) {
          break;
        }
        image_msg = latest_image_msg_;
        has_new_frame_ = false;
      }

      if (!image_msg) {
        continue;
      }

      ++frame_counter_;

      StageTimings timings;

      const bool need_visualization = (vis_pub_.getNumSubscribers() > 0);
      const bool need_victim_cloud = (victim_cloud_pub_.getNumSubscribers() > 0);
      const bool need_qr_cloud = (qr_cloud_pub_.getNumSubscribers() > 0);

      const bool need_victim_position_work = need_visualization || need_victim_cloud || publish_tf_;
      const bool need_qr_position_work = need_visualization || need_qr_cloud || publish_tf_;
      const bool need_qr_work =
          enable_qr_detection_ && (need_visualization || need_qr_cloud || publish_tf_);

      cv::Mat raw_frame;
      try {
        raw_frame = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(1.0, "cv_bridge conversion failed: %s", e.what());
        continue;
      }

      if (raw_frame.empty()) {
        continue;
      }

      const auto preprocess_start = Clock::now();

      cv::Mat corrected = applyOrientationCorrection(raw_frame);
      LetterboxResult letterbox = preprocess(corrected);
      prepareInputTensor(letterbox.image);

      const auto preprocess_end = Clock::now();
      timings.preprocess_ms = elapsedMs(preprocess_start, preprocess_end);

      std::vector<Detection> victim_detections =
          infer(corrected.size(), letterbox, timings);
      for (auto& det : victim_detections) {
        det.kind = DetectionKind::Victim;
        det.label = "victim";
      }
      filterOversizedVictimDetections(victim_detections, corrected.size());

      std::vector<Detection> qr_detections;
      bool qr_detection_executed_this_frame = false;
      if (need_qr_work) {
        const auto qr_start = Clock::now();
        qr_detections =
            getQrDetectionsOptimized(corrected, qr_detection_executed_this_frame);
        const auto qr_end = Clock::now();
        timings.qr_detect_ms = elapsedMs(qr_start, qr_end);
      }

      const auto point_fit_start = Clock::now();
      if (need_victim_position_work) {
        annotateVictimDetectionsWithPointCloud(victim_detections, corrected.size());
      }
      if (need_qr_position_work && !qr_detections.empty()) {
        annotateQrDetectionsWithPointCloud(qr_detections, corrected.size());
      }
      const auto point_fit_end = Clock::now();
      timings.point_fit_ms = elapsedMs(point_fit_start, point_fit_end);

      const std::vector<Detection> victim_publish_detections =
          gateDetectionsByConsecutiveFrames(
              victim_detections,
              true,
              victim_detect_streak_,
              victim_consecutive_frames_required_,
              victim_publish_conf_threshold_);

      const bool qr_allow_count_on_this_frame =
          enable_qr_detection_ &&
          (qr_detection_executed_this_frame || qr_reuse_cached_result_);

      const std::vector<Detection> qr_publish_detections =
          gateDetectionsByConsecutiveFrames(
              qr_detections,
              qr_allow_count_on_this_frame,
              qr_detect_streak_,
              qr_consecutive_frames_required_,
              qr_publish_conf_threshold_);

      const double fps = updateFps();

      if (need_victim_cloud) {
        const auto publish_start = Clock::now();
        publishDetectionCloud(victim_publish_detections, image_msg->header, victim_cloud_pub_);
        const auto publish_end = Clock::now();
        timings.publish_ms += elapsedMs(publish_start, publish_end);
      }

      if (need_qr_cloud) {
        const auto publish_start = Clock::now();
        publishDetectionCloud(qr_publish_detections, image_msg->header, qr_cloud_pub_);
        const auto publish_end = Clock::now();
        timings.publish_ms += elapsedMs(publish_start, publish_end);
      }

      if (publish_tf_) {
        const auto publish_start = Clock::now();
        publishDetectionTFs(victim_publish_detections, image_msg->header.stamp, victim_tf_prefix_);
        publishDetectionTFs(qr_publish_detections, image_msg->header.stamp, qr_tf_prefix_);
        const auto publish_end = Clock::now();
        timings.publish_ms += elapsedMs(publish_start, publish_end);
      }

      if (need_visualization) {
        const auto draw_start = Clock::now();
        cv::Mat vis_frame = corrected.clone();
        drawDetections(vis_frame, victim_detections, qr_detections, fps);
        const auto draw_end = Clock::now();
        timings.draw_ms = elapsedMs(draw_start, draw_end);

        const auto publish_start = Clock::now();
        publishVisualization(vis_frame, image_msg->header);
        const auto publish_end = Clock::now();
        timings.publish_ms += elapsedMs(publish_start, publish_end);
      }

      maybeLogProfiling(timings,
                        victim_detections.size(),
                        qr_detections.size(),
                        need_visualization,
                        need_victim_cloud,
                        need_qr_cloud);
    }
  }

  cv::Mat applyOrientationCorrection(const cv::Mat& frame) const {
    if (!rotate_180_) {
      return frame;
    }
    cv::Mat rotated;
    cv::rotate(frame, rotated, cv::ROTATE_180);
    return rotated;
  }

  LetterboxResult preprocess(const cv::Mat& frame) {
    LetterboxResult result;

    const float scale =
        std::min(static_cast<float>(input_size_) / static_cast<float>(frame.cols),
                 static_cast<float>(input_size_) / static_cast<float>(frame.rows));
    const int resized_w = static_cast<int>(std::round(frame.cols * scale));
    const int resized_h = static_cast<int>(std::round(frame.rows * scale));

    result.scale = scale;
    result.pad_x = (input_size_ - resized_w) / 2;
    result.pad_y = (input_size_ - resized_h) / 2;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(resized_w, resized_h), 0.0, 0.0, cv::INTER_LINEAR);

    result.image = cv::Mat(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(result.image(cv::Rect(result.pad_x, result.pad_y, resized_w, resized_h)));

    return result;
  }

  void prepareInputTensor(const cv::Mat& bgr) {
    const int width = bgr.cols;
    const int height = bgr.rows;
    const size_t plane = static_cast<size_t>(width) * static_cast<size_t>(height);

    float* dst_r = input_tensor_values_.data();
    float* dst_g = dst_r + plane;
    float* dst_b = dst_g + plane;

    for (int y = 0; y < height; ++y) {
      const cv::Vec3b* row = bgr.ptr<cv::Vec3b>(y);
      const size_t row_offset = static_cast<size_t>(y) * static_cast<size_t>(width);
      for (int x = 0; x < width; ++x) {
        const cv::Vec3b& px = row[x];
        const size_t idx = row_offset + static_cast<size_t>(x);
        dst_r[idx] = static_cast<float>(px[2]) * (1.0f / 255.0f);
        dst_g[idx] = static_cast<float>(px[1]) * (1.0f / 255.0f);
        dst_b[idx] = static_cast<float>(px[0]) * (1.0f / 255.0f);
      }
    }
  }

  std::vector<Detection> infer(const cv::Size& original_size,
                               const LetterboxResult& letterbox,
                               StageTimings& timings) {
    ov::Tensor input_tensor(
        ov::element::f32,
        input_port_.get_shape(),
        input_tensor_values_.data());

    infer_request_.set_input_tensor(input_tensor);

    const auto inference_start = Clock::now();
    infer_request_.infer();
    const auto inference_end = Clock::now();
    timings.inference_ms = elapsedMs(inference_start, inference_end);

    const ov::Tensor output_tensor = infer_request_.get_output_tensor();
    const auto output_shape = output_tensor.get_shape();
    const float* data = output_tensor.data<const float>();

    size_t element_count = 1;
    std::vector<int64_t> shape;
    shape.reserve(output_shape.size());
    for (const auto dim : output_shape) {
      element_count *= dim;
      shape.push_back(static_cast<int64_t>(dim));
    }

    const auto decode_start = Clock::now();
    std::vector<Detection> detections =
        decodeDetections(data, shape, element_count, original_size, letterbox);
    const auto decode_end = Clock::now();
    timings.decode_ms = elapsedMs(decode_start, decode_end);

    return detections;
  }

  std::vector<Detection> decodeDetections(const float* data,
                                          const std::vector<int64_t>& shape,
                                          size_t element_count,
                                          const cv::Size& original_size,
                                          const LetterboxResult& letterbox) const {
    size_t num_boxes = 0;
    bool channel_first = false;

    if (shape.size() == 3 && shape[1] == kExpectedAttributes) {
      channel_first = true;
      num_boxes = static_cast<size_t>(shape[2]);
    } else if (shape.size() == 3 && shape[2] == kExpectedAttributes) {
      channel_first = false;
      num_boxes = static_cast<size_t>(shape[1]);
    } else if (shape.size() == 2 && shape[1] == kExpectedAttributes) {
      channel_first = false;
      num_boxes = static_cast<size_t>(shape[0]);
    } else if (shape.size() == 2 && shape[0] == kExpectedAttributes) {
      channel_first = true;
      num_boxes = static_cast<size_t>(shape[1]);
    } else if (element_count % kExpectedAttributes == 0) {
      channel_first = false;
      num_boxes = element_count / kExpectedAttributes;
    } else {
      ROS_WARN_THROTTLE(1.0, "Unsupported YOLO output shape");
      return {};
    }

    auto read_value = [&](size_t attr, size_t idx) -> float {
      if (shape.size() == 3 && channel_first) {
        return data[attr * num_boxes + idx];
      }
      if (shape.size() == 3 && !channel_first) {
        return data[idx * kExpectedAttributes + attr];
      }
      if (shape.size() == 2 && channel_first) {
        return data[attr * num_boxes + idx];
      }
      return data[idx * kExpectedAttributes + attr];
    };

    struct RawCandidate {
      cv::Rect box;
      float score = 0.0f;
    };

    std::vector<RawCandidate> candidates;
    candidates.reserve(num_boxes);

    for (size_t i = 0; i < num_boxes; ++i) {
      const float cx = read_value(0, i);
      const float cy = read_value(1, i);
      const float bw = read_value(2, i);
      const float bh = read_value(3, i);

      float score = read_value(4, i);
      if (score < 0.0f || score > 1.0f) {
        score = sigmoid(score);
      }
      if (score < conf_threshold_) {
        continue;
      }

      const float x = (cx - bw * 0.5f - static_cast<float>(letterbox.pad_x)) / letterbox.scale;
      const float y = (cy - bh * 0.5f - static_cast<float>(letterbox.pad_y)) / letterbox.scale;
      const float w = bw / letterbox.scale;
      const float h = bh / letterbox.scale;

      const int x1 = std::clamp(static_cast<int>(std::round(x)), 0, original_size.width - 1);
      const int y1 = std::clamp(static_cast<int>(std::round(y)), 0, original_size.height - 1);
      const int x2 = std::clamp(static_cast<int>(std::round(x + w)), 0, original_size.width - 1);
      const int y2 = std::clamp(static_cast<int>(std::round(y + h)), 0, original_size.height - 1);

      if (x2 <= x1 || y2 <= y1) {
        continue;
      }

      RawCandidate cand;
      cand.box = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
      cand.score = score;
      candidates.push_back(cand);
    }

    if (max_candidates_pre_nms_ > 0 &&
        static_cast<int>(candidates.size()) > max_candidates_pre_nms_) {
      std::nth_element(
          candidates.begin(),
          candidates.begin() + max_candidates_pre_nms_,
          candidates.end(),
          [](const RawCandidate& a, const RawCandidate& b) {
            return a.score > b.score;
          });
      candidates.resize(static_cast<size_t>(max_candidates_pre_nms_));
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(candidates.size());
    scores.reserve(candidates.size());

    for (const auto& cand : candidates) {
      boxes.push_back(cand.box);
      scores.push_back(cand.score);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(
        boxes, scores,
        static_cast<float>(conf_threshold_),
        static_cast<float>(nms_threshold_),
        keep);

    std::vector<Detection> detections;
    detections.reserve(keep.size());
    for (const int idx : keep) {
      Detection det;
      det.box = boxes[idx];
      det.score = scores[idx];
      detections.push_back(det);
    }
    return detections;
  }

  void filterOversizedVictimDetections(std::vector<Detection>& detections,
                                       const cv::Size& image_size) const {
    if (image_size.width <= 0 || image_size.height <= 0) {
      return;
    }

    const double image_area =
        static_cast<double>(image_size.width) * static_cast<double>(image_size.height);
    if (image_area <= 1.0) {
      return;
    }

    const double max_area =
        image_area * (victim_max_bbox_area_percent_ / 100.0);

    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
                       [&](const Detection& det) {
                         const double box_area =
                             static_cast<double>(det.box.width) *
                             static_cast<double>(det.box.height);
                         return box_area > max_area;
                       }),
        detections.end());
  }

  std::vector<Detection> getQrDetectionsOptimized(const cv::Mat& frame,
                                                  bool& detection_executed_this_frame) {
    detection_executed_this_frame = false;

    if (!enable_qr_detection_) {
      return {};
    }

    const bool should_detect_now =
        (frame_counter_ == 1) ||
        ((frame_counter_ % static_cast<size_t>(qr_detect_interval_)) == 0);

    if (should_detect_now) {
      detection_executed_this_frame = true;
      last_qr_detections_cache_ = detectQrCodesScaled(frame);
      return last_qr_detections_cache_;
    }

    if (qr_reuse_cached_result_) {
      return last_qr_detections_cache_;
    }

    return {};
  }

  std::vector<Detection> detectQrCodesScaled(const cv::Mat& frame) {
    if (frame.empty()) return {};

    double scale = 1.0;
    cv::Mat detect_img = frame;

    const int max_side = std::max(frame.cols, frame.rows);
    if (max_side > qr_detect_max_side_) {
      scale = static_cast<double>(qr_detect_max_side_) / static_cast<double>(max_side);
      const int new_w = std::max(1, static_cast<int>(std::round(frame.cols * scale)));
      const int new_h = std::max(1, static_cast<int>(std::round(frame.rows * scale)));
      cv::resize(frame, detect_img, cv::Size(new_w, new_h), 0.0, 0.0, cv::INTER_AREA);
    }

    std::vector<std::array<cv::Point2f, 4>> polygons = detectQrPolygons(detect_img);
    std::vector<Detection> detections;
    detections.reserve(polygons.size());

    const bool use_reference_filter = qr_enable_reference_filter_ && !qr_references_.empty();

    for (auto polygon : polygons) {
      if (scale != 1.0) {
        for (auto& p : polygon) {
          p.x = static_cast<float>(p.x / scale);
          p.y = static_cast<float>(p.y / scale);
        }
      }

      const auto ordered = orderQuad(polygon);
      const cv::Rect box = boundingRectFromQuad(ordered, frame.size());
      if (box.width < qr_min_box_size_ || box.height < qr_min_box_size_) {
        continue;
      }

      float confidence = 0.0f;
      int matched_ref_idx = -1;

      if (use_reference_filter) {
        cv::Mat warped_binary;
        if (!warpAndNormalizeQr(frame, ordered, warped_binary)) {
          continue;
        }

        double best_score = 0.0;
        matched_ref_idx = matchQrReference(warped_binary, best_score);
        if (matched_ref_idx < 0 || best_score < qr_reference_match_threshold_) {
          continue;
        }
        confidence = static_cast<float>(best_score);
      } else {
        confidence = estimateQrGeometryConfidence(ordered, box);
      }

      Detection det;
      det.kind = DetectionKind::QRCode;
      det.label = "qrcode";
      det.box = box;
      det.score = confidence;
      det.use_center_window_average = true;
      det.center_window_size = qr_center_window_size_;
      det.qr_reference_index = matched_ref_idx;
      det.image_center = quadCenter(ordered);
      detections.push_back(det);
    }

    std::sort(detections.begin(), detections.end(),
              [](const Detection& a, const Detection& b) {
                if (a.box.x != b.box.x) return a.box.x < b.box.x;
                return a.box.y < b.box.y;
              });

    return detections;
  }

  std::vector<std::array<cv::Point2f, 4>> detectQrPolygons(const cv::Mat& frame) {
    std::vector<std::array<cv::Point2f, 4>> result;
    if (frame.empty()) return result;

    cv::Mat gray;
    if (frame.channels() == 3) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
      cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
      gray = frame;
    }

#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 3)
    cv::Mat points;
    if (!qr_detector_.detectMulti(gray, points)) {
      return result;
    }
    return parseQrPoints(points);
#else
    cv::Mat points;
    if (!qr_detector_.detect(gray, points)) {
      return result;
    }
    return parseQrPoints(points);
#endif
  }

  std::vector<std::array<cv::Point2f, 4>> parseQrPoints(const cv::Mat& points) const {
    std::vector<std::array<cv::Point2f, 4>> result;
    if (points.empty()) return result;

    cv::Mat pts = points;
    if (pts.type() != CV_32FC2) {
      pts.convertTo(pts, CV_32FC2);
    }
    if (!pts.isContinuous()) {
      pts = pts.clone();
    }

    const size_t count = pts.total();
    if (count < 4 || (count % 4) != 0) {
      return result;
    }

    const cv::Point2f* ptr = pts.ptr<cv::Point2f>(0);
    const size_t qr_count = count / 4;
    result.reserve(qr_count);

    for (size_t i = 0; i < qr_count; ++i) {
      std::array<cv::Point2f, 4> quad;
      quad[0] = ptr[i * 4 + 0];
      quad[1] = ptr[i * 4 + 1];
      quad[2] = ptr[i * 4 + 2];
      quad[3] = ptr[i * 4 + 3];
      result.push_back(quad);
    }

    return result;
  }

  std::array<cv::Point2f, 4> orderQuad(const std::array<cv::Point2f, 4>& quad) const {
    std::array<cv::Point2f, 4> out;

    std::array<float, 4> sums;
    std::array<float, 4> diffs;
    for (size_t i = 0; i < 4; ++i) {
      sums[i] = quad[i].x + quad[i].y;
      diffs[i] = quad[i].x - quad[i].y;
    }

    const int tl = static_cast<int>(std::min_element(sums.begin(), sums.end()) - sums.begin());
    const int br = static_cast<int>(std::max_element(sums.begin(), sums.end()) - sums.begin());
    const int tr = static_cast<int>(std::max_element(diffs.begin(), diffs.end()) - diffs.begin());
    const int bl = static_cast<int>(std::min_element(diffs.begin(), diffs.end()) - diffs.begin());

    out[0] = quad[tl];
    out[1] = quad[tr];
    out[2] = quad[br];
    out[3] = quad[bl];
    return out;
  }

  cv::Point2f quadCenter(const std::array<cv::Point2f, 4>& quad) const {
    cv::Point2f c(0.0f, 0.0f);
    for (const auto& p : quad) c += p;
    c.x *= 0.25f;
    c.y *= 0.25f;
    return c;
  }

  cv::Rect boundingRectFromQuad(const std::array<cv::Point2f, 4>& quad,
                                const cv::Size& image_size) const {
    float min_x = quad[0].x;
    float min_y = quad[0].y;
    float max_x = quad[0].x;
    float max_y = quad[0].y;

    for (int i = 1; i < 4; ++i) {
      min_x = std::min(min_x, quad[i].x);
      min_y = std::min(min_y, quad[i].y);
      max_x = std::max(max_x, quad[i].x);
      max_y = std::max(max_y, quad[i].y);
    }

    const int x1 = std::clamp(static_cast<int>(std::floor(min_x)), 0, image_size.width - 1);
    const int y1 = std::clamp(static_cast<int>(std::floor(min_y)), 0, image_size.height - 1);
    const int x2 = std::clamp(static_cast<int>(std::ceil(max_x)), 0, image_size.width - 1);
    const int y2 = std::clamp(static_cast<int>(std::ceil(max_y)), 0, image_size.height - 1);

    if (x2 <= x1 || y2 <= y1) {
      return cv::Rect();
    }
    return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
  }

  bool normalizeQrReferenceImage(const cv::Mat& input, cv::Mat& out_binary) const {
    if (input.empty()) return false;

    cv::Mat gray;
    if (input.channels() == 3) {
      cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    } else if (input.channels() == 4) {
      cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);
    } else {
      gray = input;
    }

    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::Mat black_mask = 255 - binary;
    std::vector<cv::Point> black_points;
    cv::findNonZero(black_mask, black_points);
    if (black_points.empty()) {
      return false;
    }

    cv::Rect bbox = cv::boundingRect(black_points);

    const int pad = 2;
    bbox.x = std::max(0, bbox.x - pad);
    bbox.y = std::max(0, bbox.y - pad);
    bbox.width = std::min(binary.cols - bbox.x, bbox.width + 2 * pad);
    bbox.height = std::min(binary.rows - bbox.y, bbox.height + 2 * pad);

    cv::Mat cropped = binary(bbox).clone();
    cv::resize(cropped, out_binary,
               cv::Size(qr_template_size_, qr_template_size_),
               0.0, 0.0, cv::INTER_NEAREST);
    return true;
  }

  bool warpAndNormalizeQr(const cv::Mat& frame,
                          const std::array<cv::Point2f, 4>& quad,
                          cv::Mat& out_binary) const {
    if (frame.empty()) return false;

    cv::Mat gray;
    if (frame.channels() == 3) {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
      cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
      gray = frame;
    }

    std::array<cv::Point2f, 4> ordered = orderQuad(quad);

    std::vector<cv::Point2f> src = {
        ordered[0], ordered[1], ordered[2], ordered[3]};
    std::vector<cv::Point2f> dst = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(qr_template_size_ - 1), 0.0f),
        cv::Point2f(static_cast<float>(qr_template_size_ - 1),
                    static_cast<float>(qr_template_size_ - 1)),
        cv::Point2f(0.0f, static_cast<float>(qr_template_size_ - 1))};

    const cv::Mat H = cv::getPerspectiveTransform(src, dst);

    cv::Mat warped;
    cv::warpPerspective(gray, warped, H,
                        cv::Size(qr_template_size_, qr_template_size_),
                        cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT,
                        cv::Scalar(255));

    return normalizeQrReferenceImage(warped, out_binary);
  }

  double binarySimilarity(const cv::Mat& a, const cv::Mat& b) const {
    if (a.empty() || b.empty()) return 0.0;
    if (a.size() != b.size() || a.type() != b.type()) return 0.0;

    cv::Mat diff;
    cv::bitwise_xor(a, b, diff);
    const double different = static_cast<double>(cv::countNonZero(diff));
    return 1.0 - different / static_cast<double>(a.total());
  }

  int matchQrReference(const cv::Mat& candidate_binary, double& best_score) const {
    best_score = 0.0;
    if (candidate_binary.empty() || qr_references_.empty()) {
      return -1;
    }

    int best_idx = -1;

    for (size_t ref_idx = 0; ref_idx < qr_references_.size(); ++ref_idx) {
      cv::Mat rotated = candidate_binary.clone();

      for (int rot = 0; rot < 4; ++rot) {
        const double score = binarySimilarity(rotated, qr_references_[ref_idx].binary);
        if (score > best_score) {
          best_score = score;
          best_idx = static_cast<int>(ref_idx);
        }

        if (rot < 3) {
          cv::Mat next;
          cv::rotate(rotated, next, cv::ROTATE_90_CLOCKWISE);
          rotated = next;
        }
      }
    }

    return best_idx;
  }

  float estimateQrGeometryConfidence(const std::array<cv::Point2f, 4>& quad,
                                     const cv::Rect& box) const {
    const float e0 = cv::norm(quad[1] - quad[0]);
    const float e1 = cv::norm(quad[2] - quad[1]);
    const float e2 = cv::norm(quad[3] - quad[2]);
    const float e3 = cv::norm(quad[0] - quad[3]);

    const float min_edge = std::max(1.0f, std::min(std::min(e0, e1), std::min(e2, e3)));
    const float max_edge = std::max(e0, std::max(e1, std::max(e2, e3)));
    const float edge_ratio = std::clamp(min_edge / std::max(1.0f, max_edge), 0.0f, 1.0f);

    const float quad_area = static_cast<float>(std::fabs(cv::contourArea(std::vector<cv::Point2f>{
        quad[0], quad[1], quad[2], quad[3]})));
    const float box_area =
        static_cast<float>(std::max(1, box.width * box.height));
    const float fill_ratio = std::clamp(quad_area / box_area, 0.0f, 1.0f);

    const float conf = 0.55f * edge_ratio + 0.45f * fill_ratio;
    return std::clamp(conf, 0.0f, 0.99f);
  }

  std::vector<Detection> collectPublishableDetections(
      const std::vector<Detection>& detections,
      double min_confidence) const {
    std::vector<Detection> out;
    out.reserve(detections.size());

    for (const auto& det : detections) {
      if (det.has_position && det.score >= min_confidence) {
        out.push_back(det);
      }
    }
    return out;
  }

  std::vector<Detection> gateDetectionsByConsecutiveFrames(
      const std::vector<Detection>& detections,
      bool allow_count_on_this_frame,
      int& streak_counter,
      int required_frames,
      double min_confidence) const {
    if (!allow_count_on_this_frame) {
      streak_counter = 0;
      return {};
    }

    const std::vector<Detection> publishable =
        collectPublishableDetections(detections, min_confidence);

    if (!publishable.empty()) {
      ++streak_counter;
    } else {
      streak_counter = 0;
    }

    if (streak_counter >= required_frames) {
      return publishable;
    }

    return {};
  }

  void annotateVictimDetectionsWithPointCloud(std::vector<Detection>& detections,
                                              const cv::Size& image_size) {
    sensor_msgs::PointCloud2ConstPtr cloud;
    CloudFieldOffsets offsets;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
      offsets = cloud_offsets_;
    }

    if (!cloud) {
      ROS_WARN_THROTTLE(2.0, "Point cloud not received yet on %s", cloud_topic_.c_str());
      return;
    }

    if (cloud->height <= 1) {
      ROS_WARN_THROTTLE(2.0,
                        "Point cloud on %s is not organized; enable ordered/aligned cloud",
                        cloud_topic_.c_str());
      return;
    }

    if (!offsets.valid()) {
      ROS_WARN_THROTTLE(2.0, "Point cloud missing x/y/z fields");
      return;
    }

    for (auto& detection : detections) {
      PointXYZ average_point;
      int support_points = 0;
      bool ok = false;

      const int area = detection.box.width * detection.box.height;
      if (area > 0 && area <= victim_small_box_area_thresh_) {
        detection.use_center_window_average = true;
        detection.image_center = cv::Point2f(
            detection.box.x + detection.box.width * 0.5f,
            detection.box.y + detection.box.height * 0.5f);
        detection.center_window_size = victim_center_window_size_;

        const cv::Point center_px =
            mapImagePointToCloud(detection.image_center, image_size, *cloud);
        ok = computeCenteredWindowPointAverage(
            *cloud, center_px, detection.center_window_size, offsets,
            victim_center_min_valid_points_, average_point, support_points);
      } else {
        const cv::Rect raw_rect = mapDetectionRectToCloud(
            detection.box, image_size, *cloud);

        ok = computeRegionPointAverage(*cloud, raw_rect, offsets,
                                       average_point, support_points);
      }

      if (ok) {
        detection.has_position = true;
        detection.position = cv::Point3f(
            average_point.x, average_point.y, average_point.z);
        detection.support_points = support_points;
      }
    }
  }

  void annotateQrDetectionsWithPointCloud(std::vector<Detection>& detections,
                                          const cv::Size& image_size) {
    sensor_msgs::PointCloud2ConstPtr cloud;
    CloudFieldOffsets offsets;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
      offsets = cloud_offsets_;
    }

    if (!cloud) {
      ROS_WARN_THROTTLE(2.0, "Point cloud not received yet on %s", cloud_topic_.c_str());
      return;
    }

    if (cloud->height <= 1) {
      ROS_WARN_THROTTLE(2.0,
                        "Point cloud on %s is not organized; enable ordered/aligned cloud",
                        cloud_topic_.c_str());
      return;
    }

    if (!offsets.valid()) {
      ROS_WARN_THROTTLE(2.0, "Point cloud missing x/y/z fields");
      return;
    }

    for (auto& detection : detections) {
      PointXYZ average_point;
      int support_points = 0;

      const cv::Point center_px =
          mapImagePointToCloud(detection.image_center, image_size, *cloud);

      if (computeCenteredWindowPointAverage(*cloud, center_px,
                                            detection.center_window_size,
                                            offsets,
                                            qr_min_valid_points_,
                                            average_point,
                                            support_points)) {
        detection.has_position = true;
        detection.position = cv::Point3f(
            average_point.x, average_point.y, average_point.z);
        detection.support_points = support_points;
      }
    }
  }

  CloudFieldOffsets extractCloudFieldOffsets(const sensor_msgs::PointCloud2& cloud) const {
    CloudFieldOffsets offsets;
    for (const auto& field : cloud.fields) {
      if (field.name == "x") {
        offsets.x = field.offset;
      } else if (field.name == "y") {
        offsets.y = field.offset;
      } else if (field.name == "z") {
        offsets.z = field.offset;
      }
    }
    return offsets;
  }

  cv::Rect mapDetectionRectToCloud(const cv::Rect& detection_box,
                                   const cv::Size& image_size,
                                   const sensor_msgs::PointCloud2& cloud) const {
    cv::Rect rect = detection_box;

    if (rotate_180_) {
      rect = cv::Rect(
          image_size.width - (detection_box.x + detection_box.width),
          image_size.height - (detection_box.y + detection_box.height),
          detection_box.width,
          detection_box.height);
    }

    const float scale_x =
        image_size.width > 0
            ? static_cast<float>(cloud.width) / static_cast<float>(image_size.width)
            : 1.0f;
    const float scale_y =
        image_size.height > 0
            ? static_cast<float>(cloud.height) / static_cast<float>(image_size.height)
            : 1.0f;

    const int x = std::clamp(
        static_cast<int>(std::floor(rect.x * scale_x)), 0,
        std::max(0, static_cast<int>(cloud.width) - 1));
    const int y = std::clamp(
        static_cast<int>(std::floor(rect.y * scale_y)), 0,
        std::max(0, static_cast<int>(cloud.height) - 1));
    const int width = std::max(1, static_cast<int>(std::ceil(rect.width * scale_x)));
    const int height = std::max(1, static_cast<int>(std::ceil(rect.height * scale_y)));
    const int max_width = std::max(1, static_cast<int>(cloud.width) - x);
    const int max_height = std::max(1, static_cast<int>(cloud.height) - y);

    return cv::Rect(x, y, std::min(width, max_width), std::min(height, max_height));
  }

  cv::Point mapImagePointToCloud(const cv::Point2f& image_point,
                                 const cv::Size& image_size,
                                 const sensor_msgs::PointCloud2& cloud) const {
    cv::Point2f pt = image_point;

    if (rotate_180_) {
      pt.x = static_cast<float>(image_size.width - 1) - pt.x;
      pt.y = static_cast<float>(image_size.height - 1) - pt.y;
    }

    const float scale_x =
        image_size.width > 0
            ? static_cast<float>(cloud.width) / static_cast<float>(image_size.width)
            : 1.0f;
    const float scale_y =
        image_size.height > 0
            ? static_cast<float>(cloud.height) / static_cast<float>(image_size.height)
            : 1.0f;

    const int u = std::clamp(
        static_cast<int>(std::round(pt.x * scale_x)),
        0, std::max(0, static_cast<int>(cloud.width) - 1));
    const int v = std::clamp(
        static_cast<int>(std::round(pt.y * scale_y)),
        0, std::max(0, static_cast<int>(cloud.height) - 1));

    return cv::Point(u, v);
  }

  bool computeRegionPointAverage(const sensor_msgs::PointCloud2& cloud,
                                 const cv::Rect& roi,
                                 const CloudFieldOffsets& offsets,
                                 PointXYZ& average_point,
                                 int& support_points) const {
    struct CandidatePoint {
      PointXYZ point;
      float depth = 0.0f;
    };

    const int step = point_sample_step_;
    const int estimated_count =
        std::max(1, (roi.width / step + 1) * (roi.height / step + 1));

    std::vector<CandidatePoint> candidates;
    candidates.reserve(static_cast<size_t>(estimated_count));

    const int max_offset = std::max({offsets.x, offsets.y, offsets.z});

    for (int v = roi.y; v < roi.y + roi.height; v += step) {
      for (int u = roi.x; u < roi.x + roi.width; u += step) {
        const size_t index =
            static_cast<size_t>(v) * cloud.row_step +
            static_cast<size_t>(u) * cloud.point_step;

        if (index + static_cast<size_t>(max_offset) + sizeof(float) > cloud.data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, &cloud.data[index + offsets.x], sizeof(float));
        std::memcpy(&y, &cloud.data[index + offsets.y], sizeof(float));
        std::memcpy(&z, &cloud.data[index + offsets.z], sizeof(float));

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z <= 0.0f) {
          continue;
        }

        candidates.push_back({PointXYZ{x, y, z}, z});
      }
    }

    if (static_cast<int>(candidates.size()) < min_valid_points_) {
      return false;
    }

    const size_t keep_count = std::max(
        static_cast<size_t>(min_valid_points_),
        static_cast<size_t>(std::round(static_cast<double>(candidates.size()) * depth_keep_ratio_)));
    const size_t clamped_keep_count = std::min(keep_count, candidates.size());

    auto cmp_depth = [](const CandidatePoint& a, const CandidatePoint& b) {
      return a.depth < b.depth;
    };

    std::nth_element(
        candidates.begin(),
        candidates.begin() + static_cast<std::ptrdiff_t>(clamped_keep_count - 1),
        candidates.end(),
        cmp_depth);

    PointXYZ sum;
    for (size_t i = 0; i < clamped_keep_count; ++i) {
      sum.x += candidates[i].point.x;
      sum.y += candidates[i].point.y;
      sum.z += candidates[i].point.z;
    }

    support_points = static_cast<int>(clamped_keep_count);
    average_point.x = sum.x / static_cast<float>(clamped_keep_count);
    average_point.y = sum.y / static_cast<float>(clamped_keep_count);
    average_point.z = sum.z / static_cast<float>(clamped_keep_count);
    return true;
  }

  bool computeCenteredWindowPointAverage(const sensor_msgs::PointCloud2& cloud,
                                         const cv::Point& center,
                                         int window_size,
                                         const CloudFieldOffsets& offsets,
                                         int required_min_valid_points,
                                         PointXYZ& average_point,
                                         int& support_points) const {
    const int half = window_size / 2;
    const int max_offset = std::max({offsets.x, offsets.y, offsets.z});

    PointXYZ sum;
    support_points = 0;

    for (int dv = -half; dv <= half; ++dv) {
      const int v = center.y + dv;
      if (v < 0 || v >= static_cast<int>(cloud.height)) continue;

      for (int du = -half; du <= half; ++du) {
        const int u = center.x + du;
        if (u < 0 || u >= static_cast<int>(cloud.width)) continue;

        const size_t index =
            static_cast<size_t>(v) * cloud.row_step +
            static_cast<size_t>(u) * cloud.point_step;

        if (index + static_cast<size_t>(max_offset) + sizeof(float) > cloud.data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, &cloud.data[index + offsets.x], sizeof(float));
        std::memcpy(&y, &cloud.data[index + offsets.y], sizeof(float));
        std::memcpy(&z, &cloud.data[index + offsets.z], sizeof(float));

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z <= 0.0f) {
          continue;
        }

        sum.x += x;
        sum.y += y;
        sum.z += z;
        ++support_points;
      }
    }

    if (support_points < required_min_valid_points) {
      return false;
    }

    average_point.x = sum.x / static_cast<float>(support_points);
    average_point.y = sum.y / static_cast<float>(support_points);
    average_point.z = sum.z / static_cast<float>(support_points);
    return true;
  }

  double updateFps() {
    const ros::Time now = ros::Time::now();
    if (!last_inference_stamp_.isZero()) {
      const double dt = (now - last_inference_stamp_).toSec();
      if (dt > 0.0) {
        const double fps = 1.0 / dt;
        smoothed_fps_ = (smoothed_fps_ == 0.0) ? fps : (0.9 * smoothed_fps_ + 0.1 * fps);
      }
    }
    last_inference_stamp_ = now;
    return smoothed_fps_;
  }

  void drawDetections(cv::Mat& frame,
                      const std::vector<Detection>& victim_detections,
                      const std::vector<Detection>& qr_detections,
                      double fps) const {
    auto draw_one = [&](const Detection& det, const cv::Scalar& color) {
      cv::rectangle(frame, det.box, color, 2);

      std::string label = cv::format("%s %.2f", det.label.c_str(), det.score);
      if (det.has_position) {
        label += cv::format(" z=%.2fm", det.position.z);
      }

      int baseline = 0;
      const cv::Size text_size =
          cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
      const int text_x = det.box.x;
      const int text_y = std::max(det.box.y - 8, text_size.height + 4);
      const cv::Rect bg_rect(
          text_x,
          text_y - text_size.height - 4,
          text_size.width + 8,
          text_size.height + 8);

      cv::rectangle(frame, bg_rect, color, cv::FILLED);
      cv::putText(frame, label, cv::Point(text_x + 4, text_y - 4),
                  cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(15, 15, 15), 2);

      if (det.has_position) {
        const std::string xyz_label = cv::format(
            "x %.2f y %.2f z %.2f n %d",
            det.position.x, det.position.y, det.position.z, det.support_points);
        cv::putText(frame, xyz_label,
                    cv::Point(det.box.x,
                              std::min(frame.rows - 8, det.box.y + det.box.height + 18)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
      }

      if (det.use_center_window_average &&
          det.image_center.x >= 0.0f && det.image_center.y >= 0.0f) {
        cv::circle(frame, det.image_center, 4, cv::Scalar(255, 180, 0), -1);
      }
    };

    for (const auto& det : victim_detections) {
      draw_one(det, cv::Scalar(0, 220, 0));
    }
    for (const auto& det : qr_detections) {
      draw_one(det, cv::Scalar(255, 0, 255));
    }

    const std::string status =
        cv::format("FPS %.1f | victim %zu [%d/%d] | qr %zu [%d/%d] | pub_v>=%.2f pub_q>=%.2f | victim_max_box<=%.1f%% | rotate180 %s",
                   fps,
                   victim_detections.size(),
                   victim_detect_streak_, victim_consecutive_frames_required_,
                   qr_detections.size(),
                   qr_detect_streak_, qr_consecutive_frames_required_,
                   victim_publish_conf_threshold_,
                   qr_publish_conf_threshold_,
                   victim_max_bbox_area_percent_,
                   rotate_180_ ? "on" : "off");
    cv::putText(frame, status, cv::Point(16, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(20, 220, 255), 2);
  }

  void publishDetectionCloud(const std::vector<Detection>& detections,
                             const std_msgs::Header& image_header,
                             ros::Publisher& pub) {
    if (pub.getNumSubscribers() == 0) {
      return;
    }

    std::vector<DetectionPoint> points;
    points.reserve(detections.size());

    for (const auto& detection : detections) {
      if (!detection.has_position) {
        continue;
      }
      points.push_back({mapPointToCameraOutput(detection.position), detection.score});
    }

    sensor_msgs::PointCloud2 out;
    out.header.stamp = image_header.stamp;
    out.header.frame_id = detection_cloud_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    out.is_bigendian = false;
    out.is_dense = false;
    out.point_step = 16;
    out.row_step = out.point_step * out.width;

    out.fields.resize(4);
    out.fields[0].name = "x";
    out.fields[0].offset = 0;
    out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    out.fields[0].count = 1;

    out.fields[1].name = "y";
    out.fields[1].offset = 4;
    out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    out.fields[1].count = 1;

    out.fields[2].name = "z";
    out.fields[2].offset = 8;
    out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    out.fields[2].count = 1;

    out.fields[3].name = "confidence";
    out.fields[3].offset = 12;
    out.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    out.fields[3].count = 1;

    out.data.resize(static_cast<size_t>(out.row_step) * out.height);
    for (size_t i = 0; i < points.size(); ++i) {
      const size_t offset = i * out.point_step;
      std::memcpy(&out.data[offset + 0],  &points[i].position.x, sizeof(float));
      std::memcpy(&out.data[offset + 4],  &points[i].position.y, sizeof(float));
      std::memcpy(&out.data[offset + 8],  &points[i].position.z, sizeof(float));
      std::memcpy(&out.data[offset + 12], &points[i].confidence, sizeof(float));
    }

    pub.publish(out);
  }

  void publishDetectionTFs(const std::vector<Detection>& detections,
                           const ros::Time& stamp,
                           const std::string& child_prefix) {
    if (!publish_tf_) return;

    std::vector<const Detection*> ordered;
    ordered.reserve(detections.size());
    for (const auto& det : detections) {
      if (det.has_position) {
        ordered.push_back(&det);
      }
    }

    std::sort(ordered.begin(), ordered.end(),
              [](const Detection* a, const Detection* b) {
                if (a->box.x != b->box.x) return a->box.x < b->box.x;
                return a->box.y < b->box.y;
              });

    for (size_t i = 0; i < ordered.size(); ++i) {
      const cv::Point3f p = mapPointToCameraOutput(ordered[i]->position);

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = tf_parent_frame_;
      tf_msg.child_frame_id = child_prefix + "_" + std::to_string(i);

      tf_msg.transform.translation.x = p.x;
      tf_msg.transform.translation.y = p.y;
      tf_msg.transform.translation.z = p.z;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_.sendTransform(tf_msg);
    }
  }

  void publishVisualization(const cv::Mat& frame,
                            const std_msgs::Header& header) {
    if (vis_pub_.getNumSubscribers() == 0) {
      return;
    }

    cv_bridge::CvImage out;
    out.header = header;
    out.encoding = "bgr8";
    out.image = frame;
    vis_pub_.publish(out.toImageMsg());
  }

  void maybeLogProfiling(const StageTimings& timings,
                         size_t victim_count,
                         size_t qr_count,
                         bool vis_enabled,
                         bool victim_cloud_enabled,
                         bool qr_cloud_enabled) {
    if (!enable_profiling_log_) {
      return;
    }

    ++profiling_frame_count_;
    profiling_acc_preprocess_ms_ += timings.preprocess_ms;
    profiling_acc_inference_ms_ += timings.inference_ms;
    profiling_acc_decode_ms_ += timings.decode_ms;
    profiling_acc_qr_detect_ms_ += timings.qr_detect_ms;
    profiling_acc_point_fit_ms_ += timings.point_fit_ms;
    profiling_acc_draw_ms_ += timings.draw_ms;
    profiling_acc_publish_ms_ += timings.publish_ms;

    if (profiling_log_interval_ <= 0 ||
        profiling_frame_count_ < static_cast<size_t>(profiling_log_interval_)) {
      return;
    }

    const double count = static_cast<double>(profiling_frame_count_);
    ROS_INFO_STREAM(
        "Profiling avg over " << profiling_frame_count_
        << " frames | preprocess=" << profiling_acc_preprocess_ms_ / count
        << " ms | inference=" << profiling_acc_inference_ms_ / count
        << " ms | decode+nms=" << profiling_acc_decode_ms_ / count
        << " ms | qr_detect=" << profiling_acc_qr_detect_ms_ / count
        << " ms | point_fit=" << profiling_acc_point_fit_ms_ / count
        << " ms | draw=" << profiling_acc_draw_ms_ / count
        << " ms | publish=" << profiling_acc_publish_ms_ / count
        << " ms | victims_last=" << victim_count
        << " | qrs_last=" << qr_count
        << " | vis=" << (vis_enabled ? "yes" : "no")
        << " | victim_cloud=" << (victim_cloud_enabled ? "yes" : "no")
        << " | qr_cloud=" << (qr_cloud_enabled ? "yes" : "no"));

    profiling_frame_count_ = 0;
    profiling_acc_preprocess_ms_ = 0.0;
    profiling_acc_inference_ms_ = 0.0;
    profiling_acc_decode_ms_ = 0.0;
    profiling_acc_qr_detect_ms_ = 0.0;
    profiling_acc_point_fit_ms_ = 0.0;
    profiling_acc_draw_ms_ = 0.0;
    profiling_acc_publish_ms_ = 0.0;
  }

  double elapsedMs(const Clock::time_point& start,
                   const Clock::time_point& end) const {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher vis_pub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher victim_cloud_pub_;
  ros::Publisher qr_cloud_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::Output<const ov::Node> input_port_;
  ov::Output<const ov::Node> output_port_;

  cv::QRCodeDetector qr_detector_;
  std::vector<QrReferenceTemplate> qr_references_;
  std::vector<Detection> last_qr_detections_cache_;

  std::vector<float> input_tensor_values_;

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  sensor_msgs::ImageConstPtr latest_image_msg_;
  bool has_new_frame_ = false;

  std::mutex cloud_mutex_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  CloudFieldOffsets cloud_offsets_;

  std::thread worker_thread_;
  std::atomic<bool> running_{true};

  std::string image_topic_;
  std::string cloud_topic_;
  std::string model_path_;
  std::string visualization_topic_;
  std::string victim_cloud_topic_;
  std::string qr_cloud_topic_;
  std::string detection_cloud_frame_;
  std::string inference_device_;
  std::string performance_mode_;

  int victim_small_box_area_thresh_ = 80 * 80;
  int victim_center_window_size_ = 9;
  int victim_center_min_valid_points_ = 8;
  double victim_max_bbox_area_percent_ = 35.0;

  int max_candidates_pre_nms_ = 200;

  bool enable_qr_detection_ = true;
  int qr_center_window_size_ = 9;
  int qr_min_valid_points_ = 8;
  int qr_min_box_size_ = 20;
  int qr_template_size_ = 256;
  bool qr_enable_reference_filter_ = true;
  double qr_reference_match_threshold_ = 0.78;
  std::string qr_reference_dir_;

  int qr_detect_interval_ = 5;
  int qr_detect_max_side_ = 640;
  bool qr_reuse_cached_result_ = true;

  bool publish_tf_ = true;
  std::string tf_parent_frame_;
  std::string victim_tf_prefix_;
  std::string qr_tf_prefix_;

  int victim_consecutive_frames_required_ = 3;
  int qr_consecutive_frames_required_ = 3;

  int victim_detect_streak_ = 0;
  int qr_detect_streak_ = 0;

  double victim_publish_conf_threshold_ = 0.65;
  double qr_publish_conf_threshold_ = 0.78;

  bool rotate_180_ = true;
  bool enable_profiling_log_ = true;

  double conf_threshold_ = 0.65;
  double nms_threshold_ = 0.45;
  int input_size_ = 640;
  int profiling_log_interval_ = 60;

  int point_sample_step_ = 2;
  int min_valid_points_ = 12;
  double depth_keep_ratio_ = 0.30;

  ros::Time last_inference_stamp_;
  double smoothed_fps_ = 0.0;

  size_t frame_counter_ = 0;
  size_t profiling_frame_count_ = 0;
  double profiling_acc_preprocess_ms_ = 0.0;
  double profiling_acc_inference_ms_ = 0.0;
  double profiling_acc_decode_ms_ = 0.0;
  double profiling_acc_qr_detect_ms_ = 0.0;
  double profiling_acc_point_fit_ms_ = 0.0;
  double profiling_acc_draw_ms_ = 0.0;
  double profiling_acc_publish_ms_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_calibration");
  try {
    VisualCalibrationNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("visual_calibration failed: " << e.what());
    return 1;
  }
  return 0;
}