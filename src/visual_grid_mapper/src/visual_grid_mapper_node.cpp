#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

namespace {
struct CalibrationPoint {
  geometry_msgs::Point point;
  float confidence = 0.0f;
};
}  // namespace

class VisualGridMapper {
 public:
  VisualGridMapper()
      : nh_(),
        pnh_("~"),
        tf_buffer_(),
        tf_listener_(tf_buffer_) {
    point_cloud_topic_ =
        pnh_.param<std::string>("point_cloud_topic", "/visual_calibration/detection_points");
    map_topic_ = pnh_.param<std::string>("map_topic", "/map");
    marker_topic_ = pnh_.param<std::string>("marker_topic", "/visual_calibration/grid_marks");
    target_frame_ = pnh_.param<std::string>("target_frame", "map");
    tf_timeout_ = ros::Duration(pnh_.param<double>("tf_timeout", 0.6));
    marker_lifetime_ = ros::Duration(pnh_.param<double>("marker_lifetime", 0.0));
    marker_z_offset_ = pnh_.param<double>("marker_z_offset", 0.03);
    dedup_radius_ = pnh_.param<double>("dedup_radius", 1.0);
    dedup_radius_sq_ = dedup_radius_ * dedup_radius_;

    map_sub_ = nh_.subscribe(map_topic_, 1, &VisualGridMapper::mapCallback, this);
    cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &VisualGridMapper::cloudCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);

    ROS_INFO_STREAM("visual_grid_mapper listening to point cloud: " << point_cloud_topic_);
    ROS_INFO_STREAM("visual_grid_mapper listening to map: " << map_topic_);
    ROS_INFO_STREAM("visual_grid_mapper publishing blue grid markers: " << marker_topic_);
    ROS_INFO_STREAM("visual_grid_mapper dedup radius: " << dedup_radius_ << " m");
  }

 private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = msg;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    nav_msgs::OccupancyGridConstPtr map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = latest_map_;
    }

    if (!map) {
      ROS_WARN_THROTTLE(2.0, "No occupancy grid received on %s yet", map_topic_.c_str());
      return;
    }

    if (map->info.resolution <= 0.0f || map->info.width == 0 || map->info.height == 0) {
      ROS_WARN_THROTTLE(2.0, "Occupancy grid on %s has invalid geometry", map_topic_.c_str());
      return;
    }

    if (!hasField(*msg, "x") || !hasField(*msg, "y") || !hasField(*msg, "z") ||
        !hasField(*msg, "confidence")) {
      ROS_WARN_THROTTLE(2.0, "Point cloud on %s is missing x/y/z/confidence fields",
                        point_cloud_topic_.c_str());
      return;
    }

    const std::string grid_frame = map->header.frame_id.empty() ? target_frame_ : map->header.frame_id;
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(grid_frame, msg->header.frame_id, msg->header.stamp, tf_timeout_);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "Failed to transform visual calibration points to %s: %s",
                        grid_frame.c_str(), ex.what());
      return;
    }

    std::vector<CalibrationPoint> frame_points;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_confidence(*msg, "confidence");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_confidence) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z) ||
          !std::isfinite(*iter_confidence)) {
        continue;
      }

      geometry_msgs::PointStamped source_point;
      source_point.header = msg->header;
      source_point.point.x = *iter_x;
      source_point.point.y = *iter_y;
      source_point.point.z = *iter_z;

      geometry_msgs::PointStamped map_point;
      tf2::doTransform(source_point, map_point, transform);

      geometry_msgs::Point grid_center;
      if (!mapPointToGridCenter(map_point.point, *map, grid_center)) {
        ROS_WARN_THROTTLE(2.0, "Visual calibration point is outside occupancy grid bounds");
        continue;
      }

      frame_points.push_back({grid_center, *iter_confidence});
    }

    for (const auto& point : frame_points) {
      updateCalibrationPoint(point);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = grid_frame;
    marker.header.stamp = msg->header.stamp;
    marker.ns = "visual_calibration_grid_marks";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map->info.resolution;
    marker.scale.y = map->info.resolution;
    marker.scale.z = std::max(0.02, static_cast<double>(map->info.resolution) * 0.2);
    marker.color.r = 0.0f;
    marker.color.g = 0.2f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.lifetime = marker_lifetime_;
    marker.points.reserve(accumulated_points_.size());
    for (const auto& point : accumulated_points_) {
      marker.points.push_back(point.point);
    }

    if (marker.points.empty()) {
      marker.action = visualization_msgs::Marker::DELETE;
    }

    marker_pub_.publish(marker);
  }

  bool mapPointToGridCenter(const geometry_msgs::Point& point, const nav_msgs::OccupancyGrid& map,
                            geometry_msgs::Point& grid_center) const {
    tf2::Transform map_origin;
    tf2::fromMsg(map.info.origin, map_origin);

    const tf2::Vector3 world_point(point.x, point.y, point.z);
    const tf2::Vector3 local_point = map_origin.inverse() * world_point;
    const double resolution = map.info.resolution;
    const double grid_x = local_point.x() / resolution;
    const double grid_y = local_point.y() / resolution;

    if (grid_x < 0.0 || grid_y < 0.0 || grid_x >= map.info.width || grid_y >= map.info.height) {
      return false;
    }

    const auto cell_x = static_cast<uint32_t>(std::floor(grid_x));
    const auto cell_y = static_cast<uint32_t>(std::floor(grid_y));
    const tf2::Vector3 local_center((static_cast<double>(cell_x) + 0.5) * resolution,
                                    (static_cast<double>(cell_y) + 0.5) * resolution,
                                    marker_z_offset_);
    const tf2::Vector3 world_center = map_origin * local_center;
    grid_center.x = world_center.x();
    grid_center.y = world_center.y();
    grid_center.z = world_center.z();
    return true;
  }

  bool hasField(const sensor_msgs::PointCloud2& cloud, const std::string& field_name) const {
    for (const auto& field : cloud.fields) {
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }

  void updateCalibrationPoint(const CalibrationPoint& candidate) {
    std::vector<size_t> matched_indices;
    for (size_t i = 0; i < accumulated_points_.size(); ++i) {
      if (distanceSquared2d(candidate.point, accumulated_points_[i].point) > dedup_radius_sq_) {
        continue;
      }
      matched_indices.push_back(i);
    }

    if (matched_indices.empty()) {
      accumulated_points_.push_back(candidate);
      return;
    }

    CalibrationPoint best = candidate;
    for (const size_t index : matched_indices) {
      if (accumulated_points_[index].confidence > best.confidence) {
        best = accumulated_points_[index];
      }
    }

    std::sort(matched_indices.rbegin(), matched_indices.rend());
    for (const size_t index : matched_indices) {
      accumulated_points_.erase(accumulated_points_.begin() + static_cast<std::ptrdiff_t>(index));
    }
    accumulated_points_.push_back(best);
  }

  double distanceSquared2d(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) const {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    return dx * dx + dy * dy;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber map_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher marker_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex map_mutex_;
  nav_msgs::OccupancyGridConstPtr latest_map_;
  std::vector<CalibrationPoint> accumulated_points_;

  std::string point_cloud_topic_;
  std::string map_topic_;
  std::string marker_topic_;
  std::string target_frame_;
  ros::Duration tf_timeout_;
  ros::Duration marker_lifetime_;
  double marker_z_offset_ = 0.03;
  double dedup_radius_ = 1.0;
  double dedup_radius_sq_ = 1.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_grid_mapper");
  VisualGridMapper node;
  ros::spin();
  return 0;
}
