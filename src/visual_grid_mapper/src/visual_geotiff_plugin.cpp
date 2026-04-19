#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <hector_geotiff/map_writer_interface.h>
#include <hector_geotiff/map_writer_plugin_interface.h>
#include <geometry_msgs/Point.h>
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
    plugin_nh.param("marker_topic", marker_topic_, std::string("/visual_calibration/grid_marks"));
    plugin_nh.param("point_label_prefix", point_label_prefix_, std::string(""));
    plugin_nh.param("draw_labels", draw_labels_, false);

    marker_sub_ = nh_.subscribe(marker_topic_, 1, &VisualCalibrationGeotiffPlugin::markerCallback, this);
    name_ = name;
    initialized_ = true;
    ROS_INFO_NAMED(name_, "Visual calibration GeoTIFF plugin listening to %s", marker_topic_.c_str());
  }

  void draw(hector_geotiff::MapWriterInterface* interface) override {
    if (!initialized_ || interface == nullptr) {
      return;
    }

    std::vector<geometry_msgs::Point> points;
    {
      std::lock_guard<std::mutex> lock(points_mutex_);
      points = points_;
    }

    const hector_geotiff::MapWriterInterface::Color blue(0, 50, 255);
    for (size_t i = 0; i < points.size(); ++i) {
      const std::string label = draw_labels_ ? point_label_prefix_ + std::to_string(i + 1) : "";
      interface->drawObjectOfInterest(Eigen::Vector2f(points[i].x, points[i].y), label, blue,
                                      hector_geotiff::SHAPE_CIRCLE);
    }

    ROS_INFO_NAMED(name_, "Drew %zu visual calibration point(s) into GeoTIFF", points.size());
  }

 private:
  void markerCallback(const visualization_msgs::MarkerConstPtr& msg) {
    if (msg->action == visualization_msgs::Marker::DELETE ||
        msg->action == visualization_msgs::Marker::DELETEALL) {
      std::lock_guard<std::mutex> lock(points_mutex_);
      points_.clear();
      return;
    }

    if (msg->type != visualization_msgs::Marker::CUBE_LIST &&
        msg->type != visualization_msgs::Marker::SPHERE_LIST &&
        msg->type != visualization_msgs::Marker::POINTS) {
      return;
    }

    std::lock_guard<std::mutex> lock(points_mutex_);
    points_ = msg->points;
  }

  ros::NodeHandle nh_;
  ros::Subscriber marker_sub_;
  std::mutex points_mutex_;
  std::vector<geometry_msgs::Point> points_;

  bool initialized_ = false;
  bool draw_labels_ = false;
  std::string name_;
  std::string marker_topic_;
  std::string point_label_prefix_;
};

}  // namespace visual_grid_mapper

PLUGINLIB_EXPORT_CLASS(visual_grid_mapper::VisualCalibrationGeotiffPlugin,
                       hector_geotiff::MapWriterPluginInterface)
