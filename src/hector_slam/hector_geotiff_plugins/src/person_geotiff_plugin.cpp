//=================================================================================================
// Copyright (c) 2026
// All rights reserved.
//=================================================================================================

#include <hector_geotiff/map_writer_interface.h>
#include <hector_geotiff/map_writer_plugin_interface.h>

#include <Eigen/Core>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace hector_geotiff_plugins
{

using namespace hector_geotiff;

class PersonMapWriter : public MapWriterPluginInterface
{
public:
  PersonMapWriter();
  virtual ~PersonMapWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface* interface);

protected:
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  std::string makeLabel(std::size_t index) const;

  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  boost::mutex mutex_;

  bool initialized_;
  std::string name_;
  std::string topic_name_;
  std::string label_prefix_;
  ros::Duration point_timeout_;
  ros::Time last_nonempty_stamp_;
  std::vector<Eigen::Vector2f> cached_points_;
  int point_color_r_;
  int point_color_g_;
  int point_color_b_;
  Shape point_shape_;
};

PersonMapWriter::PersonMapWriter()
  : initialized_(false)
  , point_timeout_(1.0)
  , point_color_r_(255)
  , point_color_g_(70)
  , point_color_b_(70)
  , point_shape_(SHAPE_CIRCLE)
{}

PersonMapWriter::~PersonMapWriter()
{}

void PersonMapWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  double point_timeout_seconds = 1.0;
  std::string point_shape_name("circle");

  plugin_nh.param("topic_name", topic_name_, std::string("/person_global_localizer/person_map_cloud"));
  plugin_nh.param("label_prefix", label_prefix_, std::string("P"));
  plugin_nh.param("point_timeout", point_timeout_seconds, 1.0);
  plugin_nh.param("point_color_r", point_color_r_, 255);
  plugin_nh.param("point_color_g", point_color_g_, 70);
  plugin_nh.param("point_color_b", point_color_b_, 70);
  plugin_nh.param("point_shape", point_shape_name, std::string("circle"));

  point_timeout_ = ros::Duration(point_timeout_seconds);
  point_shape_ = (point_shape_name == "diamond") ? SHAPE_DIAMOND : SHAPE_CIRCLE;
  point_cloud_sub_ = nh_.subscribe(topic_name_, 1, &PersonMapWriter::pointCloudCallback, this);

  initialized_ = true;
  name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized hector_geotiff MapWriter plugin %s.", name_.c_str());
}

void PersonMapWriter::draw(MapWriterInterface* interface)
{
  if (!initialized_) {
    return;
  }

  std::vector<Eigen::Vector2f> points;
  ros::Time stamp;
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    points = cached_points_;
    stamp = last_nonempty_stamp_;
  }

  if (points.empty()) {
    return;
  }

  if (!stamp.isZero() && point_timeout_.toSec() > 0.0 && (ros::Time::now() - stamp) > point_timeout_) {
    ROS_WARN_THROTTLE_NAMED(
      2.0,
      name_,
      "Skipping person overlay because the latest person center cloud is older than %.2f s.",
      point_timeout_.toSec()
    );
    return;
  }

  const MapWriterInterface::Color color(point_color_r_, point_color_g_, point_color_b_);
  for (std::size_t i = 0; i < points.size(); ++i) {
    interface->drawObjectOfInterest(points[i], makeLabel(i), color, point_shape_);
  }
}

void PersonMapWriter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std::vector<Eigen::Vector2f> points;

  if (cloud_msg->width == 0 || cloud_msg->height == 0) {
    return;
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const float x = *iter_x;
      const float y = *iter_y;

      if (std::isfinite(x) && std::isfinite(y)) {
        points.push_back(Eigen::Vector2f(x, y));
      }
    }
  } catch (const std::runtime_error& ex) {
    ROS_ERROR_THROTTLE_NAMED(2.0, name_, "Failed to parse person center cloud: %s", ex.what());
    return;
  }

  if (points.empty()) {
    return;
  }

  boost::lock_guard<boost::mutex> lock(mutex_);
  cached_points_.swap(points);
  last_nonempty_stamp_ = cloud_msg->header.stamp.isZero() ? ros::Time::now() : cloud_msg->header.stamp;
}

std::string PersonMapWriter::makeLabel(std::size_t index) const
{
  std::ostringstream stream;
  stream << label_prefix_ << (index + 1);
  return stream.str();
}

} // namespace hector_geotiff_plugins

PLUGINLIB_EXPORT_CLASS(hector_geotiff_plugins::PersonMapWriter, hector_geotiff::MapWriterPluginInterface)
