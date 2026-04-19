#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <openvino/openvino.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace fs = std::filesystem;

namespace {
constexpr int kChannels = 3;
constexpr int kExpectedAttributes = 5;

float sigmoid(float x) {
  return 1.0f / (1.0f + std::exp(-x));
}

struct LetterboxResult {
  cv::Mat image;
  float scale = 1.0f;
  int pad_x = 0;
  int pad_y = 0;
};

struct Detection {
  cv::Rect box;
  float score = 0.0f;
  bool has_position = false;
  cv::Point3f position;
  int support_points = 0;
};

struct StageTimings {
  double preprocess_ms = 0.0;
  double inference_ms = 0.0;
  double decode_ms = 0.0;
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

cv::Point3f mapPointToCameraOutput(const cv::Point3f& point) {
  // Raw fitted data: x=left, y=up, z=front.
  // Publish in camera_depth_frame semantics requested by the user:
  // right=-x, up=y, front=-z -> (-x, y, -z).
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
        core_() {
    image_topic_ = pnh_.param<std::string>("image_topic", "/camera/color/image_raw");
    cloud_topic_ = pnh_.param<std::string>("cloud_topic", "/camera/depth_registered/points");
    model_path_ = pnh_.param<std::string>("model_path", defaultModelPath());
    rotate_180_ = pnh_.param<bool>("rotate_180", true);
    visualization_topic_ =
        pnh_.param<std::string>("visualization_topic", "/visual_calibration/visualization");
    detection_cloud_topic_ =
        pnh_.param<std::string>("detection_cloud_topic", "/visual_calibration/detection_points");
    detection_cloud_frame_ =
        pnh_.param<std::string>("detection_cloud_frame", "camera_depth_frame");
    inference_device_ = pnh_.param<std::string>("inference_device", "AUTO:GPU,CPU");
    conf_threshold_ = pnh_.param<double>("conf_threshold", 0.65);
    nms_threshold_ = pnh_.param<double>("nms_threshold", 0.45);
    input_size_ = pnh_.param<int>("input_size", 640);
    enable_profiling_log_ = pnh_.param<bool>("enable_profiling_log", true);
    profiling_log_interval_ = pnh_.param<int>("profiling_log_interval", 30);

    if (!fs::exists(model_path_)) {
      ROS_FATAL_STREAM("ONNX model not found: " << model_path_);
      throw std::runtime_error("model path does not exist");
    }

    loadModel();

    image_sub_ = image_transport_.subscribe(image_topic_, 1, &VisualCalibrationNode::imageCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &VisualCalibrationNode::cloudCallback, this);
    vis_pub_ = image_transport_.advertise(visualization_topic_, 1);
    detection_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(detection_cloud_topic_, 5, true);
    input_tensor_values_.resize(static_cast<size_t>(kChannels * input_size_ * input_size_));

    worker_thread_ = std::thread(&VisualCalibrationNode::workerLoop, this);
    ROS_INFO_STREAM("visual_calibration is listening on " << image_topic_);
    ROS_INFO_STREAM("Point cloud topic: " << cloud_topic_);
    ROS_INFO_STREAM("Using model " << model_path_);
    ROS_INFO_STREAM("Visualization topic: " << visualization_topic_);
    ROS_INFO_STREAM("Detection cloud topic: " << detection_cloud_topic_);
    ROS_INFO_STREAM("Detection cloud frame: " << detection_cloud_frame_);
    ROS_INFO_STREAM("OpenVINO device: " << inference_device_);
    ROS_INFO_STREAM("Profiling log: " << (enable_profiling_log_ ? "enabled" : "disabled")
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
      auto available_devices = core_.get_available_devices();
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
    config[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::LATENCY;
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

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = frame.clone();
      latest_header_ = msg->header;
      has_new_frame_ = true;
    }
    frame_cv_.notify_one();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = msg;
  }

  void workerLoop() {
    while (ros::ok() && running_.load()) {
      cv::Mat frame;
      std_msgs::Header header;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait(lock, [this]() { return !running_.load() || has_new_frame_; });
        if (!running_.load()) {
          break;
        }
        frame = latest_frame_;
        header = latest_header_;
        has_new_frame_ = false;
      }

      if (frame.empty()) {
        continue;
      }

      StageTimings timings;
      const auto preprocess_start = Clock::now();
      cv::Mat corrected = applyOrientationCorrection(frame);
      const auto letterbox = preprocess(corrected);
      prepareInputTensor(letterbox.image);
      const auto preprocess_end = Clock::now();
      timings.preprocess_ms = elapsedMs(preprocess_start, preprocess_end);

      auto detections = infer(corrected.size(), letterbox, timings);
      const auto point_fit_start = Clock::now();
      annotateDetectionsWithPointCloud(detections, corrected.size());
      const auto point_fit_end = Clock::now();
      timings.point_fit_ms = elapsedMs(point_fit_start, point_fit_end);

      const double fps = updateFps();
      publishDetectionCloud(detections, header);
      if (vis_pub_.getNumSubscribers() > 0) {
        const auto draw_start = Clock::now();
        drawDetections(corrected, detections, fps);
        const auto draw_end = Clock::now();
        timings.draw_ms = elapsedMs(draw_start, draw_end);
        const auto publish_start = Clock::now();
        publishVisualization(corrected, header);
        const auto publish_end = Clock::now();
        timings.publish_ms = elapsedMs(publish_start, publish_end);
      }
      maybeLogProfiling(timings, detections.size(), vis_pub_.getNumSubscribers() > 0);
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

  LetterboxResult preprocess(const cv::Mat& frame) const {
    LetterboxResult result;
    const float scale =
        std::min(static_cast<float>(input_size_) / static_cast<float>(frame.cols),
                 static_cast<float>(input_size_) / static_cast<float>(frame.rows));
    const int resized_w = static_cast<int>(std::round(frame.cols * scale));
    const int resized_h = static_cast<int>(std::round(frame.rows * scale));
    result.pad_x = (input_size_ - resized_w) / 2;
    result.pad_y = (input_size_ - resized_h) / 2;
    result.scale = scale;

    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(resized_w, resized_h), 0.0, 0.0, cv::INTER_LINEAR);

    result.image = cv::Mat(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(result.image(cv::Rect(result.pad_x, result.pad_y, resized_w, resized_h)));
    return result;
  }

  void prepareInputTensor(const cv::Mat& bgr) {
    cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0, cv::Size(input_size_, input_size_),
                                          cv::Scalar(), true, false, CV_32F);
    std::memcpy(input_tensor_values_.data(), blob.ptr<float>(),
                input_tensor_values_.size() * sizeof(float));
  }

  std::vector<Detection> infer(const cv::Size& original_size, const LetterboxResult& letterbox,
                               StageTimings& timings) {
    ov::Tensor input_tensor(ov::element::f32, input_port_.get_shape(), input_tensor_values_.data());
    infer_request_.set_input_tensor(input_tensor);
    const auto inference_start = Clock::now();
    infer_request_.infer();
    const auto inference_end = Clock::now();
    timings.inference_ms = elapsedMs(inference_start, inference_end);
    const ov::Tensor output_tensor = infer_request_.get_output_tensor();
    const auto output_shape = output_tensor.get_shape();
    size_t element_count = 1;
    std::vector<int64_t> shape;
    shape.reserve(output_shape.size());
    for (const auto dim : output_shape) {
      element_count *= dim;
      shape.push_back(static_cast<int64_t>(dim));
    }
    const float* data = output_tensor.data<const float>();

    const auto decode_start = Clock::now();
    auto detections = decodeDetections(data, shape, element_count, original_size, letterbox);
    const auto decode_end = Clock::now();
    timings.decode_ms = elapsedMs(decode_start, decode_end);
    return detections;
  }

  void maybeLogProfiling(const StageTimings& timings, size_t detection_count, bool publishing) {
    if (!enable_profiling_log_) {
      return;
    }
    ++profiling_frame_count_;
    profiling_acc_preprocess_ms_ += timings.preprocess_ms;
    profiling_acc_inference_ms_ += timings.inference_ms;
    profiling_acc_decode_ms_ += timings.decode_ms;
    profiling_acc_point_fit_ms_ += timings.point_fit_ms;
    profiling_acc_draw_ms_ += timings.draw_ms;
    profiling_acc_publish_ms_ += timings.publish_ms;

    if (profiling_log_interval_ <= 0 || profiling_frame_count_ < profiling_log_interval_) {
      return;
    }

    const double count = static_cast<double>(profiling_frame_count_);
    ROS_INFO_STREAM("Profiling avg over " << profiling_frame_count_
                    << " frames | preprocess=" << profiling_acc_preprocess_ms_ / count
                    << " ms | inference=" << profiling_acc_inference_ms_ / count
                    << " ms | decode+nms=" << profiling_acc_decode_ms_ / count
                    << " ms | point_fit=" << profiling_acc_point_fit_ms_ / count
                    << " ms | draw=" << profiling_acc_draw_ms_ / count
                    << " ms | publish=" << profiling_acc_publish_ms_ / count
                    << " ms | detections_last=" << detection_count
                    << " | publishing=" << (publishing ? "yes" : "no"));

    profiling_frame_count_ = 0;
    profiling_acc_preprocess_ms_ = 0.0;
    profiling_acc_inference_ms_ = 0.0;
    profiling_acc_decode_ms_ = 0.0;
    profiling_acc_point_fit_ms_ = 0.0;
    profiling_acc_draw_ms_ = 0.0;
    profiling_acc_publish_ms_ = 0.0;
  }

  double elapsedMs(const Clock::time_point& start, const Clock::time_point& end) const {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

  std::vector<Detection> decodeDetections(const float* data, const std::vector<int64_t>& shape,
                                          size_t element_count, const cv::Size& original_size,
                                          const LetterboxResult& letterbox) const {
    size_t num_boxes = 0;
    bool channel_first = false;

    if (shape.size() == 3 && shape[1] == kExpectedAttributes) {
      channel_first = true;
      num_boxes = static_cast<size_t>(shape[2]);
    } else if (shape.size() == 3 && shape[2] == kExpectedAttributes) {
      num_boxes = static_cast<size_t>(shape[1]);
    } else if (shape.size() == 2 && shape[1] == kExpectedAttributes) {
      num_boxes = static_cast<size_t>(shape[0]);
    } else if (shape.size() == 2 && shape[0] == kExpectedAttributes) {
      channel_first = true;
      num_boxes = static_cast<size_t>(shape[1]);
    } else if (element_count % kExpectedAttributes == 0) {
      num_boxes = element_count / kExpectedAttributes;
    } else {
      ROS_WARN_THROTTLE(1.0, "Unsupported YOLO output shape");
      return {};
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(num_boxes);
    scores.reserve(num_boxes);

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

      boxes.emplace_back(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)));
      scores.emplace_back(score);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(conf_threshold_), static_cast<float>(nms_threshold_), keep);

    std::vector<Detection> detections;
    detections.reserve(keep.size());
    for (const int idx : keep) {
      detections.push_back({boxes[idx], scores[idx]});
    }
    return detections;
  }

  void annotateDetectionsWithPointCloud(std::vector<Detection>& detections, const cv::Size& image_size) {
    sensor_msgs::PointCloud2ConstPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
    }
    if (!cloud) {
      ROS_WARN_THROTTLE(2.0, "Point cloud not received yet on %s", cloud_topic_.c_str());
      return;
    }
    if (cloud->height <= 1) {
      ROS_WARN_THROTTLE(2.0, "Point cloud on %s is not organized; enable ordered_pc and aligned colored point cloud",
                        cloud_topic_.c_str());
      return;
    }

    int offset_x = -1;
    int offset_y = -1;
    int offset_z = -1;
    for (const auto& field : cloud->fields) {
      if (field.name == "x") {
        offset_x = field.offset;
      } else if (field.name == "y") {
        offset_y = field.offset;
      } else if (field.name == "z") {
        offset_z = field.offset;
      }
    }
    if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
      ROS_WARN_THROTTLE(2.0, "Point cloud missing x/y/z fields");
      return;
    }

    for (auto& detection : detections) {
      const cv::Rect raw_rect = mapDetectionRectToCloud(detection.box, image_size, *cloud);
      PointXYZ average_point;
      int support_points = 0;
      if (computeRegionPointAverage(*cloud, raw_rect, offset_x, offset_y, offset_z, average_point, support_points)) {
        detection.has_position = true;
        detection.position = cv::Point3f(average_point.x, average_point.y, average_point.z);
        detection.support_points = support_points;
      }
    }
  }

  cv::Rect mapDetectionRectToCloud(const cv::Rect& detection_box, const cv::Size& image_size,
                                   const sensor_msgs::PointCloud2& cloud) const {
    cv::Rect rect = detection_box;
    if (rotate_180_) {
      rect = cv::Rect(image_size.width - (detection_box.x + detection_box.width),
                      image_size.height - (detection_box.y + detection_box.height), detection_box.width,
                      detection_box.height);
    }

    const float scale_x =
        image_size.width > 0 ? static_cast<float>(cloud.width) / static_cast<float>(image_size.width) : 1.0f;
    const float scale_y =
        image_size.height > 0 ? static_cast<float>(cloud.height) / static_cast<float>(image_size.height) : 1.0f;

    const int x = std::clamp(static_cast<int>(std::floor(rect.x * scale_x)), 0,
                             std::max(0, static_cast<int>(cloud.width) - 1));
    const int y = std::clamp(static_cast<int>(std::floor(rect.y * scale_y)), 0,
                             std::max(0, static_cast<int>(cloud.height) - 1));
    const int width = std::max(1, static_cast<int>(std::ceil(rect.width * scale_x)));
    const int height = std::max(1, static_cast<int>(std::ceil(rect.height * scale_y)));
    const int max_width = std::max(1, static_cast<int>(cloud.width) - x);
    const int max_height = std::max(1, static_cast<int>(cloud.height) - y);
    return cv::Rect(x, y, std::min(width, max_width), std::min(height, max_height));
  }

  bool computeRegionPointAverage(const sensor_msgs::PointCloud2& cloud, const cv::Rect& roi, int offset_x,
                                 int offset_y, int offset_z, PointXYZ& average_point,
                                 int& support_points) const {
    struct CandidatePoint {
      PointXYZ point;
      float metric = 0.0f;
    };

    std::vector<CandidatePoint> candidates;
    candidates.reserve(static_cast<size_t>(roi.width * roi.height));

    for (int v = roi.y; v < roi.y + roi.height; ++v) {
      for (int u = roi.x; u < roi.x + roi.width; ++u) {
        const size_t index = static_cast<size_t>(v) * cloud.row_step + static_cast<size_t>(u) * cloud.point_step;
        if (index + static_cast<size_t>(std::max({offset_x, offset_y, offset_z})) + sizeof(float) > cloud.data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, &cloud.data[index + offset_x], sizeof(float));
        std::memcpy(&y, &cloud.data[index + offset_y], sizeof(float));
        std::memcpy(&z, &cloud.data[index + offset_z], sizeof(float));

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z <= 0.0f) {
          continue;
        }

        candidates.push_back({PointXYZ{x, y, z}, z});
      }
    }

    if (candidates.size() < 10) {
      return false;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const CandidatePoint& lhs, const CandidatePoint& rhs) { return lhs.metric < rhs.metric; });

    const size_t count = candidates.size();
    size_t begin = static_cast<size_t>(count * 0.2);
    size_t end = static_cast<size_t>(count * 0.4);
    end = std::min(end, count);
    if (end <= begin) {
      return false;
    }

    PointXYZ sum;
    for (size_t i = begin; i < end; ++i) {
      sum.x += candidates[i].point.x;
      sum.y += candidates[i].point.y;
      sum.z += candidates[i].point.z;
    }

    support_points = static_cast<int>(end - begin);
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
        smoothed_fps_ = smoothed_fps_ == 0.0 ? fps : (0.9 * smoothed_fps_ + 0.1 * fps);
      }
    }
    last_inference_stamp_ = now;
    return smoothed_fps_;
  }

  void drawDetections(cv::Mat& frame, const std::vector<Detection>& detections, double fps) const {
    for (const auto& det : detections) {
      cv::rectangle(frame, det.box, cv::Scalar(0, 220, 0), 2);
      std::string label = cv::format("obj %.2f", det.score);
      if (det.has_position) {
        label += cv::format(" z=%.2fm", det.position.z);
      }
      int baseline = 0;
      const cv::Size text_size =
          cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
      const int text_x = det.box.x;
      const int text_y = std::max(det.box.y - 8, text_size.height + 4);
      const cv::Rect bg_rect(text_x, text_y - text_size.height - 4, text_size.width + 8,
                             text_size.height + 8);
      cv::rectangle(frame, bg_rect, cv::Scalar(0, 220, 0), cv::FILLED);
      cv::putText(frame, label, cv::Point(text_x + 4, text_y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  cv::Scalar(15, 15, 15), 2);
      if (det.has_position) {
        const std::string xyz_label =
            cv::format("x %.2f y %.2f z %.2f n %d", det.position.x, det.position.y, det.position.z,
                       det.support_points);
        cv::putText(frame, xyz_label, cv::Point(det.box.x, std::min(frame.rows - 8, det.box.y + det.box.height + 18)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
      }
    }

    const std::string status = cv::format("FPS %.1f | det %zu | rotate180 %s", fps, detections.size(),
                                          rotate_180_ ? "on" : "off");
    cv::putText(frame, status, cv::Point(16, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(20, 220, 255), 2);
  }

  void publishDetectionCloud(const std::vector<Detection>& detections, const std_msgs::Header& image_header) {
    std::vector<DetectionPoint> points;
    points.reserve(detections.size());
    for (const auto& detection : detections) {
      if (!detection.has_position) {
        continue;
      }
      points.push_back({mapPointToCameraOutput(detection.position), detection.score});
    }

    const std::string frame_id = detection_cloud_frame_;

    sensor_msgs::PointCloud2 out;
    out.header.stamp = image_header.stamp;
    out.header.frame_id = frame_id;
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
      std::memcpy(&out.data[offset + 0], &points[i].position.x, sizeof(float));
      std::memcpy(&out.data[offset + 4], &points[i].position.y, sizeof(float));
      std::memcpy(&out.data[offset + 8], &points[i].position.z, sizeof(float));
      std::memcpy(&out.data[offset + 12], &points[i].confidence, sizeof(float));
    }

    detection_cloud_pub_.publish(out);
  }

  void publishVisualization(const cv::Mat& frame, const std_msgs::Header& header) {
    if (vis_pub_.getNumSubscribers() == 0) {
      return;
    }
    cv_bridge::CvImage out;
    out.header = header;
    out.encoding = "bgr8";
    out.image = frame;
    vis_pub_.publish(out.toImageMsg());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher vis_pub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher detection_cloud_pub_;

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::Output<const ov::Node> input_port_;
  ov::Output<const ov::Node> output_port_;
  std::vector<float> input_tensor_values_;

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  cv::Mat latest_frame_;
  std_msgs::Header latest_header_;
  bool has_new_frame_ = false;
  std::mutex cloud_mutex_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;

  std::thread worker_thread_;
  std::atomic<bool> running_{true};

  std::string image_topic_;
  std::string cloud_topic_;
  std::string model_path_;
  std::string visualization_topic_;
  std::string detection_cloud_topic_;
  std::string detection_cloud_frame_;
  std::string inference_device_;
  bool rotate_180_ = true;
  bool enable_profiling_log_ = true;
  double conf_threshold_ = 0.65;
  double nms_threshold_ = 0.45;
  int input_size_ = 640;
  int profiling_log_interval_ = 30;
  ros::Time last_inference_stamp_;
  double smoothed_fps_ = 0.0;
  size_t profiling_frame_count_ = 0;
  double profiling_acc_preprocess_ms_ = 0.0;
  double profiling_acc_inference_ms_ = 0.0;
  double profiling_acc_decode_ms_ = 0.0;
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
