#include <algorithm>
#include <mutex>
#include <set>
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

    const std::string default_victim_label_prefix =
        legacy_label_prefix.empty() ? std::string("Person") : legacy_label_prefix;
    plugin_nh.param("victim_label_prefix", victim_label_prefix_, default_victim_label_prefix);
    plugin_nh.param("qr_label_prefix", qr_label_prefix_, std::string("QRCode"));

    plugin_nh.param("draw_labels", draw_labels_, true);
    dedup_radius_ = plugin_nh.param("dedup_radius", 1.0);
    dedup_radius_sq_ = dedup_radius_ * dedup_radius_;

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

    std::vector<LabeledPoint> victim_points;
    std::vector<LabeledPoint> qr_points;
    {
      std::lock_guard<std::mutex> lock(points_mutex_);
      victim_points = victim_points_;
      qr_points = qr_points_;
    }

    const hector_geotiff::MapWriterInterface::Color blue(0, 50, 255);
    const hector_geotiff::MapWriterInterface::Color red(255, 0, 0);

    for (const auto& point : victim_points) {
      const std::string label = draw_labels_
                                    ? (victim_label_prefix_ + std::to_string(point.label_id))
                                    : "";
      interface->drawObjectOfInterest(
          Eigen::Vector2f(point.point.x, point.point.y),
          label, blue, hector_geotiff::SHAPE_CIRCLE);
    }

    for (const auto& point : qr_points) {
      const std::string label =
          draw_labels_ ? (qr_label_prefix_ + std::to_string(point.label_id)) : "";
      interface->drawObjectOfInterest(
          Eigen::Vector2f(point.point.x, point.point.y),
          label, red, hector_geotiff::SHAPE_CIRCLE);
    }

    ROS_INFO_NAMED(name_.c_str(),
                   "Drew %zu victim point(s) and %zu QR point(s) into GeoTIFF",
                   victim_points.size(), qr_points.size());
  }

 private:
  struct LabeledPoint {
    geometry_msgs::Point point;
    size_t label_id = 0;
  };

  void victimMarkerCallback(const visualization_msgs::MarkerConstPtr& msg) {
    updatePointsFromMarker(msg, victim_points_, next_victim_label_id_);
  }

  void qrMarkerCallback(const visualization_msgs::MarkerConstPtr& msg) {
    updatePointsFromMarker(msg, qr_points_, next_qr_label_id_);
  }

  void updatePointsFromMarker(const visualization_msgs::MarkerConstPtr& msg,
                              std::vector<LabeledPoint>& storage,
                              size_t& next_label_id) {
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

    std::vector<LabeledPoint> next_points;
    next_points.reserve(msg->points.size());
    std::vector<bool> old_used(storage.size(), false);

    for (const auto& candidate : msg->points) {
      int best_index = -1;
      double best_distance_sq = dedup_radius_sq_;

      for (size_t i = 0; i < storage.size(); ++i) {
        if (old_used[i]) {
          continue;
        }
        const double distance_sq = distanceSquared2d(candidate, storage[i].point);
        if (distance_sq <= best_distance_sq) {
          best_distance_sq = distance_sq;
          best_index = static_cast<int>(i);
        }
      }

      if (best_index >= 0) {
        old_used[static_cast<size_t>(best_index)] = true;
        LabeledPoint kept = storage[static_cast<size_t>(best_index)];
        kept.point = candidate;
        next_points.push_back(kept);
        continue;
      }

      next_points.push_back(LabeledPoint{candidate, next_label_id++});
    }

    normalizeLabels(next_points);
    storage = std::move(next_points);
  }

  static double distanceSquared2d(const geometry_msgs::Point& lhs,
                                  const geometry_msgs::Point& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    return dx * dx + dy * dy;
  }

  static void normalizeLabels(std::vector<LabeledPoint>& points) {
    std::set<size_t> sorted_labels;
    for (const auto& point : points) {
      sorted_labels.insert(point.label_id);
    }

    std::vector<size_t> labels(sorted_labels.begin(), sorted_labels.end());
    for (auto& point : points) {
      const auto iter = std::lower_bound(labels.begin(), labels.end(), point.label_id);
      if (iter == labels.end()) {
        continue;
      }
      point.label_id =
          static_cast<size_t>(std::distance(labels.begin(), iter)) + static_cast<size_t>(1);
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber victim_marker_sub_;
  ros::Subscriber qr_marker_sub_;

  std::mutex points_mutex_;
  std::vector<LabeledPoint> victim_points_;
  std::vector<LabeledPoint> qr_points_;

  bool initialized_ = false;
  bool draw_labels_ = false;
  size_t next_victim_label_id_ = 1;
  size_t next_qr_label_id_ = 1;
  double dedup_radius_ = 1.0;
  double dedup_radius_sq_ = 1.0;

  std::string name_;
  std::string victim_marker_topic_;
  std::string qr_marker_topic_;
  std::string victim_label_prefix_;
  std::string qr_label_prefix_;
};

}  // namespace visual_grid_mapper

PLUGINLIB_EXPORT_CLASS(visual_grid_mapper::VisualCalibrationGeotiffPlugin,
                       hector_geotiff::MapWriterPluginInterface)
