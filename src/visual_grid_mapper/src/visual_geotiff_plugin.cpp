#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <hector_geotiff/map_writer_interface.h>
#include <hector_geotiff/map_writer_plugin_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace visual_grid_mapper {

class VisualCalibrationGeotiffPlugin : public hector_geotiff::MapWriterPluginInterface {
 public:
  VisualCalibrationGeotiffPlugin() = default;
  ~VisualCalibrationGeotiffPlugin() = default;

  void initialize(const std::string& name) override {
    ros::NodeHandle plugin_nh("~/" + name);

    // 兼容旧参数：marker_topic 继续作为受困者 marker 话题
    std::string legacy_marker_topic = "/visual_calibration/grid_marks";
    plugin_nh.param("marker_topic", legacy_marker_topic, legacy_marker_topic);

    std::string legacy_label_prefix;
    plugin_nh.param("point_label_prefix", legacy_label_prefix, std::string(""));

    plugin_nh.param("victim_marker_topic", victim_marker_topic_, legacy_marker_topic);
    plugin_nh.param("qr_marker_topic", qr_marker_topic_, std::string("/visual_calibration/qr_grid_marks"));

    plugin_nh.param("victim_label_prefix", victim_label_prefix_, legacy_label_prefix);
    plugin_nh.param("qr_label_prefix", qr_label_prefix_, std::string("Q"));

    plugin_nh.param("draw_labels", draw_labels_, false);

    victim_marker_sub_ =
        nh_.subscribe(victim_marker_topic_, 1,
                      &VisualCalibrationGeotiffPlugin::victimMarkerCallback, this);
    qr_marker_sub_ =
        nh_.subscribe(qr_marker_topic_, 1,
                      &VisualCalibrationGeotiffPlugin::qrMarkerCallback, this);

    name_ = name;
    initialized_ = true;

    ROS_INFO_NAMED(name_.c_str(),
                   "VisualCalibrationGeotiffPlugin listening to victim markers: %s",
                   victim_marker_topic_.c_str());
    ROS_INFO_NAMED(name_.c_str(),
                   "VisualCalibrationGeotiffPlugin listening to QR markers: %s",
                   qr_marker_topic_.c_str());
  }

  void draw(hector_geotiff::MapWriterInterface* interface) override {
    if (!initialized_ || interface == nullptr) {
      return;
    }

    std::vector<geometry_msgs::Point> victim_points;
    std::vector<geometry_msgs::Point> qr_points;
    {
      std::lock_guard<std::mutex> lock(points_mutex_);
      victim_points = victim_points_;
      qr_points = qr_points_;
    }

    const hector_geotiff::MapWriterInterface::Color blue(0, 50, 255);
    const hector_geotiff::MapWriterInterface::Color red(255, 0, 0);

    for (size_t i = 0; i < victim_points.size(); ++i) {
      const std::string label =
          draw_labels_ ? (victim_label_prefix_ + std::to_string(i + 1)) : "";
      interface->drawObjectOfInterest(
          Eigen::Vector2f(victim_points[i].x, victim_points[i].y),
          label, blue, hector_geotiff::SHAPE_CIRCLE);
    }

    for (size_t i = 0; i < qr_points.size(); ++i) {
      const std::string label =
          draw_labels_ ? (qr_label_prefix_ + std::to_string(i + 1)) : "";
      interface->drawObjectOfInterest(
          Eigen::Vector2f(qr_points[i].x, qr_points[i].y),
          label, red, hector_geotiff::SHAPE_CIRCLE);
    }

    ROS_INFO_NAMED(name_.c_str(),
                   "Drew %zu victim point(s) and %zu QR point(s) into GeoTIFF",
                   victim_points.size(), qr_points.size());
  }

 private:
  void victimMarkerCallback(const visualization_msgs::MarkerConstPtr& msg) {
    updatePointsFromMarker(msg, victim_points_);
  }

  void qrMarkerCallback(const visualization_msgs::MarkerConstPtr& msg) {
    updatePointsFromMarker(msg, qr_points_);
  }

  void updatePointsFromMarker(const visualization_msgs::MarkerConstPtr& msg,
                              std::vector<geometry_msgs::Point>& storage) {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(points_mutex_);

    if (msg->action == visualization_msgs::Marker::DELETE ||
        msg->action == visualization_msgs::Marker::DELETEALL) {
      storage.clear();
      return;
    }

    if (msg->type != visualization_msgs::Marker::CUBE_LIST &&
        msg->type != visualization_msgs::Marker::SPHERE_LIST &&
        msg->type != visualization_msgs::Marker::POINTS) {
      return;
    }

    storage = msg->points;
  }

  ros::NodeHandle nh_;
  ros::Subscriber victim_marker_sub_;
  ros::Subscriber qr_marker_sub_;

  std::mutex points_mutex_;
  std::vector<geometry_msgs::Point> victim_points_;
  std::vector<geometry_msgs::Point> qr_points_;

  bool initialized_ = false;
  bool draw_labels_ = false;

  std::string name_;
  std::string victim_marker_topic_;
  std::string qr_marker_topic_;
  std::string victim_label_prefix_;
  std::string qr_label_prefix_;
};

}  // namespace visual_grid_mapper

PLUGINLIB_EXPORT_CLASS(visual_grid_mapper::VisualCalibrationGeotiffPlugin,
                       hector_geotiff::MapWriterPluginInterface)