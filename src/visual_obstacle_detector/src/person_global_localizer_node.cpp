#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{

struct XYZIPoint
{
  float x;
  float y;
  float z;
  float intensity;
};

struct Detection
{
  cv::Rect bbox;
  cv::Point2f center;
  float confidence;
  int confirm_count;
  bool is_confirmed;
  bool newly_confirmed;
};

struct Track
{
  cv::Point2f center;
  int count;
  double stamp;
};

struct LetterboxResult
{
  cv::Mat image;
  float scale;
  int pad_x;
  int pad_y;
};

inline int clampInt(const int value, const int low, const int high)
{
  return std::max(low, std::min(high, value));
}

inline float clampFloat(const float value, const float low, const float high)
{
  return std::max(low, std::min(high, value));
}

std::string toLowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

}  // namespace

class PersonGlobalLocalizerNode
{
public:
  PersonGlobalLocalizerNode()
    : nh_()
    , pnh_("~")
    , tf_buffer_()
    , tf_listener_(tf_buffer_)
  {
    const std::string default_model_path =
      ros::package::getPath("visual_obstacle_detector") + "/models/yolo.onnx";

    pnh_.param("model_path", model_path_, default_model_path);
    pnh_.param("color_topic", color_topic_, std::string("/camera/color/image_raw"));
    pnh_.param("depth_topic", depth_topic_, std::string("/camera/aligned_depth_to_color/image_raw"));
    pnh_.param("camera_info_topic", camera_info_topic_, std::string("/camera/color/camera_info"));
    pnh_.param("conf_threshold", conf_threshold_, 0.5);
    pnh_.param("iou_threshold", iou_threshold_, 0.45);
    pnh_.param("sync_slop", sync_slop_, 0.1);
    pnh_.param("depth_unit_scale", depth_unit_scale_, 0.001);
    pnh_.param("depth_min", depth_min_, 0.2);
    pnh_.param("depth_max", depth_max_, 8.0);
    pnh_.param("depth_roi_half", depth_roi_half_, 4);
    pnh_.param("marker_lifetime", marker_lifetime_, 0.3);
    pnh_.param("publish_debug_image", publish_debug_image_, true);
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("transform_timeout", transform_timeout_, 0.05);
    pnh_.param("min_confirm_frames", min_confirm_frames_, 3);
    pnh_.param("confirm_pixel_tolerance", confirm_pixel_tolerance_, 60.0);
    pnh_.param("confirm_timeout", confirm_timeout_, 0.5);
    pnh_.param("input_width", input_width_, 640);
    pnh_.param("input_height", input_height_, 640);
    pnh_.param("compute_target", compute_target_, std::string("auto"));

    min_confirm_frames_ = std::max(1, min_confirm_frames_);
    depth_roi_half_ = std::max(1, depth_roi_half_);
    input_width_ = std::max(32, input_width_);
    input_height_ = std::max(32, input_height_);

    ROS_INFO("Loading ONNX person model from %s", model_path_.c_str());
    net_ = cv::dnn::readNetFromONNX(model_path_);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(resolveDnnTarget());

    person_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("person_camera_cloud", 1);
    person_map_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("person_map_cloud", 1);
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("person_markers", 1);
    if (publish_debug_image_) {
      debug_image_pub_ = pnh_.advertise<sensor_msgs::Image>("debug_image", 1);
    }

    color_sub_.subscribe(nh_, color_topic_, 1);
    depth_sub_.subscribe(nh_, depth_topic_, 1);
    info_sub_.subscribe(nh_, camera_info_topic_, 1);
    sync_.reset(new Sync(SyncPolicy(10), color_sub_, depth_sub_, info_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(sync_slop_));
    sync_->registerCallback(
      boost::bind(
        &PersonGlobalLocalizerNode::syncedCallback,
        this,
        boost::placeholders::_1,
        boost::placeholders::_2,
        boost::placeholders::_3));

    ROS_INFO(
      "person_global_localizer C++ node is ready, publishing person centers in %s after %d consecutive frames",
      global_frame_.c_str(),
      min_confirm_frames_);
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  int resolveDnnTarget()
  {
    const std::string normalized_target = toLowerCopy(compute_target_);
    const bool wants_opencl =
      normalized_target == "opencl" ||
      normalized_target == "opencl_fp16" ||
      normalized_target == "auto";

    if (wants_opencl) {
      cv::ocl::setUseOpenCL(true);
    }

    const bool opencl_available = cv::ocl::haveOpenCL();
    const bool opencl_enabled = cv::ocl::useOpenCL();
    if (opencl_available && opencl_enabled) {
      const cv::ocl::Device device = cv::ocl::Device::getDefault();
      if (device.available()) {
        ROS_INFO(
          "OpenCL device detected for person detector: %s | vendor=%s | version=%s",
          device.name().c_str(),
          device.vendorName().c_str(),
          device.version().c_str());
      }
    }

    if (normalized_target == "cpu") {
      ROS_INFO("Person detector compute target: CPU");
      return cv::dnn::DNN_TARGET_CPU;
    }

    if (normalized_target == "opencl_fp16") {
      if (opencl_available && opencl_enabled) {
        ROS_INFO("Person detector compute target: OpenCL FP16");
        return cv::dnn::DNN_TARGET_OPENCL_FP16;
      }
      ROS_WARN("Requested OpenCL FP16 for person detector, but no OpenCL runtime/device is available. Falling back to CPU.");
      return cv::dnn::DNN_TARGET_CPU;
    }

    if (normalized_target == "opencl") {
      if (opencl_available && opencl_enabled) {
        ROS_INFO("Person detector compute target: OpenCL");
        return cv::dnn::DNN_TARGET_OPENCL;
      }
      ROS_WARN("Requested OpenCL for person detector, but no OpenCL runtime/device is available. Falling back to CPU.");
      return cv::dnn::DNN_TARGET_CPU;
    }

    if (normalized_target == "auto") {
      if (opencl_available && opencl_enabled) {
        ROS_INFO("Person detector compute target: AUTO -> OpenCL");
        return cv::dnn::DNN_TARGET_OPENCL;
      }
      ROS_WARN("Person detector compute target AUTO could not enable OpenCL. Falling back to CPU.");
      return cv::dnn::DNN_TARGET_CPU;
    }

    ROS_WARN(
      "Unknown person detector compute_target [%s]. Supported values: auto, cpu, opencl, opencl_fp16. Falling back to CPU.",
      compute_target_.c_str());
    return cv::dnn::DNN_TARGET_CPU;
  }

  void syncedCallback(
    const sensor_msgs::ImageConstPtr& color_msg,
    const sensor_msgs::ImageConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
  {
    cv_bridge::CvImageConstPtr color_bridge;
    cv_bridge::CvImageConstPtr depth_bridge;
    try {
      color_bridge = cv_bridge::toCvShare(color_msg, "bgr8");
      depth_bridge = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    } catch (const cv_bridge::Exception& ex) {
      ROS_WARN_THROTTLE(2.0, "cv_bridge conversion failed: %s", ex.what());
      return;
    }

    cv::Mat depth_m;
    if (!depthToMeters(depth_bridge->image, depth_m)) {
      ROS_WARN_THROTTLE(2.0, "Unsupported depth image encoding for person localizer: %s", depth_msg->encoding.c_str());
      return;
    }

    std::vector<Detection> detections = runInference(color_bridge->image);
    updateConfirmations(detections, color_msg->header.stamp.toSec());

    std::vector<XYZIPoint> current_camera_points;
    std::vector<XYZIPoint> current_map_points;
    visualization_msgs::MarkerArray marker_array;
    cv::Mat debug_image = color_bridge->image.clone();
    const std::string camera_frame = depth_msg->header.frame_id;
    int confirmed_marker_id = 0;

    for (std::size_t i = 0; i < detections.size(); ++i) {
      Detection& detection = detections[i];

      cv::Point pixel_xy = bboxCenterPixel(detection.bbox, depth_m.size());
      float depth_value = 0.0f;
      bool has_depth = false;
      geometry_msgs::PointStamped camera_point;
      bool has_camera_point = false;

      if (detection.newly_confirmed) {
        ROS_INFO(
          "Person detection confirmed after %d consecutive frames near pixel (%d, %d)",
          detection.confirm_count,
          pixel_xy.x,
          pixel_xy.y);
      }

      if (detection.is_confirmed) {
        cv::Point sampled_pixel;
        cv::Point3f point_xyz;
        if (sampleDepthAndPoint(depth_m, detection.bbox, *camera_info_msg, point_xyz, sampled_pixel, depth_value)) {
          pixel_xy = sampled_pixel;
          camera_point.header = color_msg->header;
          camera_point.header.frame_id = camera_frame;
          camera_point.point.x = point_xyz.x;
          camera_point.point.y = point_xyz.y;
          camera_point.point.z = point_xyz.z;
          has_camera_point = true;
          has_depth = true;

          current_camera_points.push_back(
            XYZIPoint{point_xyz.x, point_xyz.y, point_xyz.z, detection.confidence});

          appendMarkers(
            marker_array,
            confirmed_marker_id,
            camera_point,
            detection.confidence,
            detection.confirm_count,
            camera_frame);

          geometry_msgs::PointStamped map_point;
          if (transformPoint(camera_point, map_point)) {
            current_map_points.push_back(
              XYZIPoint{
                static_cast<float>(map_point.point.x),
                static_cast<float>(map_point.point.y),
                static_cast<float>(map_point.point.z),
                detection.confidence});
          }

          ++confirmed_marker_id;
        }
      }

      drawDetection(
        debug_image,
        detection,
        pixel_xy,
        has_depth,
        depth_value,
        has_camera_point,
        camera_point,
        has_camera_point ? confirmed_marker_id - 1 : -1);
    }

    publishCloud(person_cloud_pub_, color_msg->header, camera_frame, current_camera_points);
    publishCloud(person_map_cloud_pub_, color_msg->header, global_frame_, current_map_points);
    publishMarkers(marker_array, color_msg->header, camera_frame);

    if (publish_debug_image_) {
      sensor_msgs::ImagePtr debug_msg =
        cv_bridge::CvImage(color_msg->header, "bgr8", debug_image).toImageMsg();
      debug_image_pub_.publish(debug_msg);
    }
  }

  bool depthToMeters(const cv::Mat& depth_image, cv::Mat& depth_m) const
  {
    if (depth_image.type() == CV_16UC1) {
      depth_image.convertTo(depth_m, CV_32FC1, depth_unit_scale_);
      return true;
    }
    if (depth_image.type() == CV_32FC1) {
      depth_m = depth_image.clone();
      return true;
    }
    if (depth_image.type() == CV_64FC1) {
      depth_image.convertTo(depth_m, CV_32FC1);
      return true;
    }
    return false;
  }

  std::vector<Detection> runInference(const cv::Mat& color_image)
  {
    LetterboxResult letterboxed = letterboxImage(color_image);

    cv::Mat blob = cv::dnn::blobFromImage(
      letterboxed.image,
      1.0 / 255.0,
      cv::Size(input_width_, input_height_),
      cv::Scalar(),
      true,
      false);

    net_.setInput(blob);
    cv::Mat output = net_.forward();

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;

    if (output.dims == 3 && output.size[1] == 5) {
      const int count = output.size[2];
      cv::Mat view(output.size[1], output.size[2], CV_32F, output.ptr<float>());
      for (int i = 0; i < count; ++i) {
        const float score = view.at<float>(4, i);
        if (score < static_cast<float>(conf_threshold_)) {
          continue;
        }
        appendDecodedBox(view.at<float>(0, i), view.at<float>(1, i), view.at<float>(2, i), view.at<float>(3, i),
          score, letterboxed, color_image.size(), boxes, scores);
      }
    } else if (output.dims == 3 && output.size[2] == 5) {
      const int count = output.size[1];
      cv::Mat view(output.size[1], output.size[2], CV_32F, output.ptr<float>());
      for (int i = 0; i < count; ++i) {
        const float score = view.at<float>(i, 4);
        if (score < static_cast<float>(conf_threshold_)) {
          continue;
        }
        appendDecodedBox(view.at<float>(i, 0), view.at<float>(i, 1), view.at<float>(i, 2), view.at<float>(i, 3),
          score, letterboxed, color_image.size(), boxes, scores);
      }
    } else {
      ROS_ERROR_THROTTLE(2.0, "Unexpected ONNX output shape for person detector");
      return std::vector<Detection>();
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(conf_threshold_), static_cast<float>(iou_threshold_), indices);

    std::vector<Detection> detections;
    detections.reserve(indices.size());
    for (std::size_t i = 0; i < indices.size(); ++i) {
      const int index = indices[i];
      Detection detection;
      detection.bbox = boxes[index];
      detection.center = cv::Point2f(
        detection.bbox.x + detection.bbox.width * 0.5f,
        detection.bbox.y + detection.bbox.height * 0.5f);
      detection.confidence = scores[index];
      detection.confirm_count = 1;
      detection.is_confirmed = false;
      detection.newly_confirmed = false;
      detections.push_back(detection);
    }

    return detections;
  }

  LetterboxResult letterboxImage(const cv::Mat& image) const
  {
    const float scale = std::min(
      static_cast<float>(input_width_) / static_cast<float>(image.cols),
      static_cast<float>(input_height_) / static_cast<float>(image.rows));
    const int resized_width = std::max(1, static_cast<int>(std::round(image.cols * scale)));
    const int resized_height = std::max(1, static_cast<int>(std::round(image.rows * scale)));

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_width, resized_height));

    const int pad_x = (input_width_ - resized_width) / 2;
    const int pad_y = (input_height_ - resized_height) / 2;
    const int right = input_width_ - resized_width - pad_x;
    const int bottom = input_height_ - resized_height - pad_y;

    cv::Mat output;
    cv::copyMakeBorder(
      resized,
      output,
      pad_y,
      bottom,
      pad_x,
      right,
      cv::BORDER_CONSTANT,
      cv::Scalar(114, 114, 114));

    LetterboxResult result;
    result.image = output;
    result.scale = scale;
    result.pad_x = pad_x;
    result.pad_y = pad_y;
    return result;
  }

  void appendDecodedBox(
    const float cx,
    const float cy,
    const float width,
    const float height,
    const float score,
    const LetterboxResult& letterboxed,
    const cv::Size& original_size,
    std::vector<cv::Rect>& boxes,
    std::vector<float>& scores) const
  {
    const float x1 = (cx - width * 0.5f - static_cast<float>(letterboxed.pad_x)) / letterboxed.scale;
    const float y1 = (cy - height * 0.5f - static_cast<float>(letterboxed.pad_y)) / letterboxed.scale;
    const float x2 = (cx + width * 0.5f - static_cast<float>(letterboxed.pad_x)) / letterboxed.scale;
    const float y2 = (cy + height * 0.5f - static_cast<float>(letterboxed.pad_y)) / letterboxed.scale;

    const int left = clampInt(static_cast<int>(std::round(x1)), 0, original_size.width - 1);
    const int top = clampInt(static_cast<int>(std::round(y1)), 0, original_size.height - 1);
    const int right = clampInt(static_cast<int>(std::round(x2)), 0, original_size.width - 1);
    const int bottom = clampInt(static_cast<int>(std::round(y2)), 0, original_size.height - 1);

    if (right <= left || bottom <= top) {
      return;
    }

    boxes.push_back(cv::Rect(left, top, right - left, bottom - top));
    scores.push_back(score);
  }

  void updateConfirmations(std::vector<Detection>& detections, const double stamp_sec)
  {
    std::vector<Track> active_tracks;
    std::vector<bool> used_tracks(pending_confirmations_.size(), false);

    for (std::size_t i = 0; i < detections.size(); ++i) {
      Detection& detection = detections[i];
      int matched_index = -1;
      float matched_distance = 0.0f;

      for (std::size_t j = 0; j < pending_confirmations_.size(); ++j) {
        if (used_tracks[j]) {
          continue;
        }

        const Track& track = pending_confirmations_[j];
        if (stamp_sec - track.stamp > confirm_timeout_) {
          continue;
        }

        const float pixel_shift = cv::norm(detection.center - track.center);
        if (pixel_shift > static_cast<float>(confirm_pixel_tolerance_)) {
          continue;
        }

        if (matched_index < 0 || pixel_shift < matched_distance) {
          matched_index = static_cast<int>(j);
          matched_distance = pixel_shift;
        }
      }

      if (matched_index < 0) {
        detection.confirm_count = 1;
      } else {
        detection.confirm_count = pending_confirmations_[matched_index].count + 1;
        used_tracks[matched_index] = true;
      }

      detection.is_confirmed = detection.confirm_count >= min_confirm_frames_;
      detection.newly_confirmed = detection.confirm_count == min_confirm_frames_;

      Track track;
      track.center = detection.center;
      track.count = detection.confirm_count;
      track.stamp = stamp_sec;
      active_tracks.push_back(track);
    }

    pending_confirmations_.swap(active_tracks);
  }

  bool sampleDepthAndPoint(
    const cv::Mat& depth_m,
    const cv::Rect& bbox,
    const sensor_msgs::CameraInfo& camera_info_msg,
    cv::Point3f& point_xyz,
    cv::Point& pixel_xy,
    float& depth_value) const
  {
    pixel_xy = bboxCenterPixel(bbox, depth_m.size());

    const int u0 = std::max(0, pixel_xy.x - depth_roi_half_);
    const int u1 = std::min(depth_m.cols, pixel_xy.x + depth_roi_half_ + 1);
    const int v0 = std::max(0, pixel_xy.y - depth_roi_half_);
    const int v1 = std::min(depth_m.rows, pixel_xy.y + depth_roi_half_ + 1);

    std::vector<float> valid_depths;
    valid_depths.reserve((u1 - u0) * (v1 - v0));
    for (int v = v0; v < v1; ++v) {
      const float* row_ptr = depth_m.ptr<float>(v);
      for (int u = u0; u < u1; ++u) {
        const float z = row_ptr[u];
        if (std::isfinite(z) && z > depth_min_ && z < depth_max_) {
          valid_depths.push_back(z);
        }
      }
    }

    if (valid_depths.empty()) {
      return false;
    }

    std::nth_element(
      valid_depths.begin(),
      valid_depths.begin() + valid_depths.size() / 2,
      valid_depths.end());
    depth_value = valid_depths[valid_depths.size() / 2];

    const double fx = camera_info_msg.K[0];
    const double fy = camera_info_msg.K[4];
    const double cx = camera_info_msg.K[2];
    const double cy = camera_info_msg.K[5];
    if (fx == 0.0 || fy == 0.0) {
      return false;
    }

    point_xyz.x = static_cast<float>((pixel_xy.x - cx) * depth_value / fx);
    point_xyz.y = static_cast<float>((pixel_xy.y - cy) * depth_value / fy);
    point_xyz.z = depth_value;
    return true;
  }

  cv::Point bboxCenterPixel(const cv::Rect& bbox, const cv::Size& image_size) const
  {
    const int u = clampInt(bbox.x + bbox.width / 2, 0, image_size.width - 1);
    const int v = clampInt(bbox.y + bbox.height / 2, 0, image_size.height - 1);
    return cv::Point(u, v);
  }

  bool transformPoint(
    const geometry_msgs::PointStamped& camera_point,
    geometry_msgs::PointStamped& map_point)
  {
    try {
      map_point = tf_buffer_.transform(
        camera_point,
        global_frame_,
        ros::Duration(transform_timeout_));
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(
        2.0,
        "failed to transform person center from %s to %s: %s",
        camera_point.header.frame_id.c_str(),
        global_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  void appendMarkers(
    visualization_msgs::MarkerArray& marker_array,
    const int marker_id,
    const geometry_msgs::PointStamped& camera_point,
    const float confidence,
    const int confirm_count,
    const std::string& camera_frame) const
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = camera_frame;
    sphere.header.stamp = camera_point.header.stamp;
    sphere.ns = "person_points";
    sphere.id = marker_id;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.orientation.w = 1.0;
    sphere.pose.position = camera_point.point;
    sphere.scale.x = 0.25;
    sphere.scale.y = 0.25;
    sphere.scale.z = 0.25;
    sphere.color = makeColor(1.0f, 0.1f, 0.1f, 0.9f);
    sphere.lifetime = ros::Duration(marker_lifetime_);
    marker_array.markers.push_back(sphere);

    const double range_dist = std::sqrt(
      camera_point.point.x * camera_point.point.x +
      camera_point.point.y * camera_point.point.y +
      camera_point.point.z * camera_point.point.z);

    visualization_msgs::Marker text;
    text.header.frame_id = camera_frame;
    text.header.stamp = camera_point.header.stamp;
    text.ns = "person_labels";
    text.id = 10000 + marker_id;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.pose.position.x = camera_point.point.x;
    text.pose.position.y = camera_point.point.y;
    text.pose.position.z = camera_point.point.z + 0.45;
    text.scale.z = 0.22;
    text.color = makeColor(0.1f, 1.0f, 0.1f, 0.95f);
    text.text = "person " + std::to_string(marker_id) + " " +
      formatFloat(range_dist) + "m " + formatFloat(confidence) + " " +
      std::to_string(std::min(confirm_count, min_confirm_frames_)) + "/" +
      std::to_string(min_confirm_frames_);
    text.lifetime = ros::Duration(marker_lifetime_);
    marker_array.markers.push_back(text);
  }

  void drawDetection(
    cv::Mat& image,
    const Detection& detection,
    const cv::Point& pixel_xy,
    const bool has_depth,
    const float depth_value,
    const bool has_camera_point,
    const geometry_msgs::PointStamped& camera_point,
    const int marker_id) const
  {
    cv::rectangle(image, detection.bbox, cv::Scalar(40, 220, 40), 2);
    cv::circle(image, pixel_xy, 4, cv::Scalar(0, 0, 255), -1);

    const std::string confirm_text =
      std::to_string(std::min(detection.confirm_count, min_confirm_frames_)) + "/" +
      std::to_string(min_confirm_frames_);
    const std::string status_text = detection.is_confirmed ? "ok" : "wait";

    std::string camera_text;
    if (has_camera_point) {
      camera_text = "id:" + std::to_string(marker_id) +
        " conf:" + formatFloat(detection.confidence) +
        " " + status_text + " " + confirm_text +
        " cam[" + formatFloat(camera_point.point.x) + "," +
        formatFloat(camera_point.point.y) + "," +
        formatFloat(camera_point.point.z) + "]m";
    } else {
      camera_text = "conf:" + formatFloat(detection.confidence) + " " + status_text + " " + confirm_text;
    }

    const std::string depth_text = has_depth ? ("depth:" + formatFloat(depth_value) + "m") : "depth:pending";

    const int y_text_1 = std::max(20, detection.bbox.y - 10);
    const int y_text_2 = std::min(image.rows - 10, y_text_1 + 20);
    cv::putText(
      image,
      camera_text,
      cv::Point(detection.bbox.x, y_text_1),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(20, 255, 20),
      1,
      cv::LINE_AA);
    cv::putText(
      image,
      depth_text,
      cv::Point(detection.bbox.x, y_text_2),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(20, 255, 255),
      1,
      cv::LINE_AA);
  }

  void publishMarkers(
    visualization_msgs::MarkerArray& marker_array,
    const std_msgs::Header& header,
    const std::string& frame_id)
  {
    visualization_msgs::Marker delete_marker;
    delete_marker.header = header;
    delete_marker.header.frame_id = frame_id;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.insert(marker_array.markers.begin(), delete_marker);
    marker_pub_.publish(marker_array);
  }

  void publishCloud(
    const ros::Publisher& publisher,
    const std_msgs::Header& header,
    const std::string& frame_id,
    const std::vector<XYZIPoint>& points)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = header;
    cloud_msg.header.frame_id = frame_id;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

    for (std::size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
      *iter_x = points[i].x;
      *iter_y = points[i].y;
      *iter_z = points[i].z;
      *iter_i = points[i].intensity;
    }

    publisher.publish(cloud_msg);
  }

  std_msgs::ColorRGBA makeColor(const float r, const float g, const float b, const float a) const
  {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  std::string formatFloat(const double value) const
  {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return std::string(buffer);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string model_path_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string global_frame_;
  std::string compute_target_;

  double conf_threshold_;
  double iou_threshold_;
  double sync_slop_;
  double depth_unit_scale_;
  double depth_min_;
  double depth_max_;
  double marker_lifetime_;
  double transform_timeout_;
  double confirm_pixel_tolerance_;
  double confirm_timeout_;
  int depth_roi_half_;
  int min_confirm_frames_;
  int input_width_;
  int input_height_;
  bool publish_debug_image_;

  cv::dnn::Net net_;
  std::vector<Track> pending_confirmations_;

  ros::Publisher person_cloud_pub_;
  ros::Publisher person_map_cloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher debug_image_pub_;

  message_filters::Subscriber<sensor_msgs::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  std::unique_ptr<Sync> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_global_localizer");

  try {
    PersonGlobalLocalizerNode node;
    ros::spin();
  } catch (const cv::Exception& ex) {
    ROS_FATAL("Failed to start person_global_localizer_node: %s", ex.what());
    return 1;
  } catch (const std::exception& ex) {
    ROS_FATAL("Failed to start person_global_localizer_node: %s", ex.what());
    return 1;
  }

  return 0;
}
