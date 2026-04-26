#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/bind/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{

struct Track
{
  cv::Point2f center;
  int count;
  double stamp;
};

inline int clampInt(const int value, const int low, const int high)
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

class QRGlobalLocalizerNode
{
public:
  QRGlobalLocalizerNode()
    : nh_()
    , pnh_("~")
    , use_opencl_(false)
    , tf_buffer_()
    , tf_listener_(tf_buffer_)
  {
    pnh_.param("color_topic", color_topic_, std::string("/camera/color/image_raw"));
    pnh_.param("depth_topic", depth_topic_, std::string("/camera/aligned_depth_to_color/image_raw"));
    pnh_.param("camera_info_topic", camera_info_topic_, std::string("/camera/color/camera_info"));
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
    pnh_.param("confirm_pixel_tolerance", confirm_pixel_tolerance_, 40.0);
    pnh_.param("confirm_timeout", confirm_timeout_, 0.5);
    pnh_.param("compute_target", compute_target_, std::string("auto"));

    min_confirm_frames_ = std::max(1, min_confirm_frames_);
    depth_roi_half_ = std::max(1, depth_roi_half_);
    configureComputeTarget();

    map_marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("qr_map_markers", 1);
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
        &QRGlobalLocalizerNode::syncedCallback,
        this,
        boost::placeholders::_1,
        boost::placeholders::_2,
        boost::placeholders::_3));

    ROS_INFO(
      "qr_global_localizer C++ node is ready, publishing single QR map markers in %s after %d consecutive frames",
      global_frame_.c_str(),
      min_confirm_frames_);
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  void configureComputeTarget()
  {
    const std::string normalized_target = toLowerCopy(compute_target_);

    if (normalized_target == "cpu") {
      cv::ocl::setUseOpenCL(false);
      use_opencl_ = false;
      ROS_INFO("QR detector compute target: CPU");
      return;
    }

    if (
      normalized_target == "gpu" ||
      normalized_target == "opencl" ||
      normalized_target == "opencl_fp16" ||
      normalized_target == "auto")
    {
      cv::ocl::setUseOpenCL(true);
      const bool opencl_available = cv::ocl::haveOpenCL();
      const bool opencl_enabled = cv::ocl::useOpenCL();
      if (opencl_available && opencl_enabled) {
        use_opencl_ = true;
        const cv::ocl::Device device = cv::ocl::Device::getDefault();
        if (device.available()) {
          ROS_INFO(
            "QR detector compute target: %s -> OpenCL device=%s vendor=%s version=%s",
            normalized_target == "auto" ? "AUTO" : "GPU",
            device.name().c_str(),
            device.vendorName().c_str(),
            device.version().c_str());
        } else {
          ROS_INFO(
            "QR detector compute target: %s -> OpenCL enabled",
            normalized_target == "auto" ? "AUTO" : "GPU");
        }
        return;
      }

      use_opencl_ = false;
      if (normalized_target == "auto") {
        ROS_WARN("QR detector compute target AUTO could not enable OpenCL. Falling back to CPU.");
      } else {
        ROS_WARN(
          "Requested QR detector GPU/OpenCL target, but no OpenCL runtime/device is available. Falling back to CPU.");
      }
      return;
    }

    cv::ocl::setUseOpenCL(true);
    use_opencl_ = cv::ocl::haveOpenCL() && cv::ocl::useOpenCL();
    ROS_WARN(
      "Unknown QR detector compute_target [%s]. Supported values: auto, gpu, cpu. Legacy values opencl/opencl_fp16 map to gpu. Falling back to %s.",
      compute_target_.c_str(),
      use_opencl_ ? "OpenCL" : "CPU");
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
      ROS_WARN_THROTTLE(2.0, "Unsupported depth image encoding for QR localizer: %s", depth_msg->encoding.c_str());
      return;
    }

    visualization_msgs::MarkerArray marker_array;
    cv::Mat debug_image = color_bridge->image.clone();
    const std::string camera_frame = depth_msg->header.frame_id;
    const double stamp_sec = color_msg->header.stamp.toSec();

    cv::Mat points;
    std::string raw_text;
    if (use_opencl_) {
      cv::UMat color_input = color_bridge->image.getUMat(cv::ACCESS_READ);
      cv::UMat gray_input;
      cv::cvtColor(color_input, gray_input, cv::COLOR_BGR2GRAY);
      raw_text = qr_detector_.detectAndDecode(gray_input, points);
    } else {
      cv::Mat gray_input;
      cv::cvtColor(color_bridge->image, gray_input, cv::COLOR_BGR2GRAY);
      raw_text = qr_detector_.detectAndDecode(gray_input, points);
    }
    const std::string qr_text = normalizeQrText(raw_text);

    std::unordered_map<std::string, bool> active_qr_texts;
    int confirm_count = 0;
    bool is_confirmed = false;
    bool newly_confirmed = false;
    std::vector<cv::Point2f> corners;

    if (!qr_text.empty() && extractCorners(points, corners)) {
      active_qr_texts[qr_text] = true;
      const cv::Point2f center = meanPoint(corners);
      updateConfirmation(qr_text, center, stamp_sec, confirm_count, is_confirmed, newly_confirmed);

      if (newly_confirmed) {
        ROS_INFO("QR %s confirmed after %d consecutive frames", qr_text.c_str(), confirm_count);
      }

      cv::Point pixel_xy;
      float depth_value = 0.0f;
      bool has_depth = false;

      if (is_confirmed) {
        cv::Point3f point_xyz;
        if (sampleDepthAndPoint(depth_m, center, *camera_info_msg, point_xyz, pixel_xy, depth_value)) {
          geometry_msgs::PointStamped camera_point;
          camera_point.header = color_msg->header;
          camera_point.header.frame_id = camera_frame;
          camera_point.point.x = point_xyz.x;
          camera_point.point.y = point_xyz.y;
          camera_point.point.z = point_xyz.z;

          geometry_msgs::PointStamped map_point;
          if (transformPoint(camera_point, map_point)) {
            appendMarkers(marker_array, 0, map_point, makeQrLabel(qr_text));
          }
          has_depth = true;
        } else {
          pixel_xy = centerPixel(center, depth_m.size());
        }
      } else {
        pixel_xy = centerPixel(center, depth_m.size());
      }

      drawDetection(
        debug_image,
        corners,
        qr_text,
        pixel_xy,
        has_depth,
        depth_value,
        confirm_count,
        is_confirmed);
    }

    cleanupConfirmations(active_qr_texts, stamp_sec);
    publishMarkers(marker_array, color_msg->header, global_frame_);

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

  std::string normalizeQrText(const std::string& raw_text) const
  {
    if (raw_text.empty()) {
      return std::string();
    }

    std::string cleaned;
    cleaned.reserve(raw_text.size());
    bool last_was_space = false;
    for (std::size_t i = 0; i < raw_text.size(); ++i) {
      const unsigned char ch = static_cast<unsigned char>(raw_text[i]);
      if (ch == 0 || ch == 127) {
        continue;
      }
      if (ch < 32) {
        if (!last_was_space) {
          cleaned.push_back(' ');
          last_was_space = true;
        }
        continue;
      }
      if (std::isspace(ch) != 0) {
        if (!last_was_space) {
          cleaned.push_back(' ');
          last_was_space = true;
        }
        continue;
      }
      cleaned.push_back(static_cast<char>(ch));
      last_was_space = false;
    }

    if (!cleaned.empty() && cleaned.front() == ' ') {
      cleaned.erase(cleaned.begin());
    }
    if (!cleaned.empty() && cleaned.back() == ' ') {
      cleaned.pop_back();
    }

    if (cleaned != raw_text) {
      ROS_WARN_THROTTLE(
        2.0,
        "QR decoded text normalized from [%s] to [%s]",
        raw_text.c_str(),
        cleaned.c_str());
    }

    return cleaned;
  }

  bool extractCorners(const cv::Mat& points, std::vector<cv::Point2f>& corners) const
  {
    if (points.empty() || points.total() < 4) {
      return false;
    }

    cv::Mat reshaped = points.reshape(1, 4);
    if (reshaped.cols < 2) {
      return false;
    }

    corners.clear();
    corners.reserve(4);
    for (int i = 0; i < 4; ++i) {
      corners.push_back(cv::Point2f(reshaped.at<float>(i, 0), reshaped.at<float>(i, 1)));
    }
    return true;
  }

  cv::Point2f meanPoint(const std::vector<cv::Point2f>& corners) const
  {
    cv::Point2f center(0.0f, 0.0f);
    for (std::size_t i = 0; i < corners.size(); ++i) {
      center += corners[i];
    }
    return center * (1.0f / static_cast<float>(corners.size()));
  }

  cv::Point centerPixel(const cv::Point2f& center_xy, const cv::Size& image_size) const
  {
    return cv::Point(
      clampInt(static_cast<int>(std::round(center_xy.x)), 0, image_size.width - 1),
      clampInt(static_cast<int>(std::round(center_xy.y)), 0, image_size.height - 1));
  }

  void updateConfirmation(
    const std::string& qr_text,
    const cv::Point2f& center_xy,
    const double stamp_sec,
    int& confirm_count,
    bool& is_confirmed,
    bool& newly_confirmed)
  {
    confirm_count = 1;

    const std::unordered_map<std::string, Track>::const_iterator it = pending_confirmations_.find(qr_text);
    if (it != pending_confirmations_.end()) {
      const Track& track = it->second;
      const double time_gap = stamp_sec - track.stamp;
      const float pixel_shift = cv::norm(center_xy - track.center);
      if (time_gap <= confirm_timeout_ && pixel_shift <= static_cast<float>(confirm_pixel_tolerance_)) {
        confirm_count = track.count + 1;
      }
    }

    Track track;
    track.center = center_xy;
    track.count = confirm_count;
    track.stamp = stamp_sec;
    pending_confirmations_[qr_text] = track;

    is_confirmed = confirm_count >= min_confirm_frames_;
    newly_confirmed = confirm_count == min_confirm_frames_;
  }

  void cleanupConfirmations(
    const std::unordered_map<std::string, bool>& active_qr_texts,
    const double stamp_sec)
  {
    std::vector<std::string> stale_keys;
    for (std::unordered_map<std::string, Track>::const_iterator it = pending_confirmations_.begin();
      it != pending_confirmations_.end();
      ++it) {
      if (active_qr_texts.find(it->first) == active_qr_texts.end() ||
          stamp_sec - it->second.stamp > confirm_timeout_) {
        stale_keys.push_back(it->first);
      }
    }

    for (std::size_t i = 0; i < stale_keys.size(); ++i) {
      pending_confirmations_.erase(stale_keys[i]);
    }
  }

  bool sampleDepthAndPoint(
    const cv::Mat& depth_m,
    const cv::Point2f& center_xy,
    const sensor_msgs::CameraInfo& camera_info_msg,
    cv::Point3f& point_xyz,
    cv::Point& pixel_xy,
    float& depth_value) const
  {
    pixel_xy = centerPixel(center_xy, depth_m.size());

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
        "failed to transform QR center from %s to %s: %s",
        camera_point.header.frame_id.c_str(),
        global_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  void appendMarkers(
    visualization_msgs::MarkerArray& marker_array,
    const int marker_id,
    const geometry_msgs::PointStamped& map_point,
    const std::string& label) const
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = global_frame_;
    sphere.header.stamp = map_point.header.stamp;
    sphere.ns = "qr_points_map";
    sphere.id = marker_id;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.orientation.w = 1.0;
    sphere.pose.position = map_point.point;
    sphere.scale.x = 0.22;
    sphere.scale.y = 0.22;
    sphere.scale.z = 0.22;
    sphere.color = makeColor(0.1f, 0.6f, 1.0f, 0.9f);
    sphere.lifetime = ros::Duration(marker_lifetime_);
    marker_array.markers.push_back(sphere);

    visualization_msgs::Marker text;
    text.header.frame_id = global_frame_;
    text.header.stamp = map_point.header.stamp;
    text.ns = "qr_labels_map";
    text.id = 10000 + marker_id;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.pose.position.x = map_point.point.x;
    text.pose.position.y = map_point.point.y;
    text.pose.position.z = map_point.point.z + 0.40;
    text.scale.z = 0.22;
    text.color = makeColor(1.0f, 1.0f, 1.0f, 0.95f);
    text.text = label;
    text.lifetime = ros::Duration(marker_lifetime_);
    marker_array.markers.push_back(text);
  }

  void drawDetection(
    cv::Mat& image,
    const std::vector<cv::Point2f>& corners,
    const std::string& qr_text,
    const cv::Point& pixel_xy,
    const bool has_depth,
    const float depth_value,
    const int confirm_count,
    const bool is_confirmed) const
  {
    std::vector<cv::Point> polygon;
    polygon.reserve(corners.size());
    for (std::size_t i = 0; i < corners.size(); ++i) {
      polygon.push_back(cv::Point(
        static_cast<int>(std::round(corners[i].x)),
        static_cast<int>(std::round(corners[i].y))));
    }

    const cv::Point* polygon_ptr = polygon.data();
    const int polygon_size = static_cast<int>(polygon.size());
    cv::polylines(image, &polygon_ptr, &polygon_size, 1, true, cv::Scalar(255, 180, 0), 2);
    cv::circle(image, pixel_xy, 4, cv::Scalar(0, 0, 255), -1);

    std::string label_text = makeQrLabel(qr_text) + " " +
      std::to_string(std::min(confirm_count, min_confirm_frames_)) + "/" +
      std::to_string(min_confirm_frames_);
    if (is_confirmed) {
      label_text += " ok";
    }

    int min_x = polygon.front().x;
    int min_y = polygon.front().y;
    for (std::size_t i = 1; i < polygon.size(); ++i) {
      min_x = std::min(min_x, polygon[i].x);
      min_y = std::min(min_y, polygon[i].y);
    }
    const int y_text_1 = std::max(20, min_y - 10);
    const int y_text_2 = std::min(image.rows - 10, y_text_1 + 20);

    cv::putText(
      image,
      label_text,
      cv::Point(min_x, y_text_1),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(255, 220, 80),
      1,
      cv::LINE_AA);

    if (has_depth) {
      cv::putText(
        image,
        "depth:" + formatFloat(depth_value) + "m",
        cv::Point(min_x, y_text_2),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(20, 255, 255),
        1,
        cv::LINE_AA);
    }
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
    map_marker_pub_.publish(marker_array);
  }

  std::string makeQrLabel(const std::string& qr_text) const
  {
    return std::string("cn") + qr_text;
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

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string global_frame_;
  std::string compute_target_;

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
  bool publish_debug_image_;
  bool use_opencl_;

  cv::QRCodeDetector qr_detector_;
  std::unordered_map<std::string, Track> pending_confirmations_;

  ros::Publisher map_marker_pub_;
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
  ros::init(argc, argv, "qr_global_localizer");
  QRGlobalLocalizerNode node;
  ros::spin();
  return 0;
}
