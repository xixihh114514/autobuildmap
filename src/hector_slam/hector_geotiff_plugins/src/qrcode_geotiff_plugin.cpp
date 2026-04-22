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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <string>
#include <vector>

namespace hector_geotiff_plugins
{

using namespace hector_geotiff;

class QRCodeMapWriter : public MapWriterPluginInterface
{
public:
  QRCodeMapWriter();
  virtual ~QRCodeMapWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface* interface);

protected:
  struct StoredLabel
  {
    Eigen::Vector2f point;
    std::string label;
    std::size_t color_index;
  };

  void markerCallback(const visualization_msgs::MarkerArrayConstPtr& marker_array);
  bool isDuplicatePoint(
    const Eigen::Vector2f& point,
    const std::vector<StoredLabel>& existing_labels
  ) const;
  MapWriterInterface::Color makeColor(std::size_t index) const;

  ros::NodeHandle nh_;
  ros::Subscriber marker_sub_;
  boost::mutex mutex_;

  bool initialized_;
  std::string name_;
  std::string topic_name_;
  double min_separation_distance_;
  double min_separation_distance_squared_;
  bool use_palette_colors_;
  int point_color_r_;
  int point_color_g_;
  int point_color_b_;
  Shape point_shape_;
  std::vector<StoredLabel> stored_labels_;
};

QRCodeMapWriter::QRCodeMapWriter()
  : initialized_(false)
  , min_separation_distance_(1.0)
  , min_separation_distance_squared_(1.0)
  , use_palette_colors_(true)
  , point_color_r_(66)
  , point_color_g_(135)
  , point_color_b_(245)
  , point_shape_(SHAPE_DIAMOND)
{}

QRCodeMapWriter::~QRCodeMapWriter()
{}

void QRCodeMapWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string point_shape_name("diamond");

  plugin_nh.param("topic_name", topic_name_, std::string("/qr_global_localizer/qr_map_markers"));
  plugin_nh.param("min_separation_distance", min_separation_distance_, 1.0);
  plugin_nh.param("use_palette_colors", use_palette_colors_, true);
  plugin_nh.param("point_color_r", point_color_r_, 66);
  plugin_nh.param("point_color_g", point_color_g_, 135);
  plugin_nh.param("point_color_b", point_color_b_, 245);
  plugin_nh.param("point_shape", point_shape_name, std::string("diamond"));

  min_separation_distance_ = std::max(0.0, min_separation_distance_);
  min_separation_distance_squared_ = min_separation_distance_ * min_separation_distance_;
  point_shape_ = (point_shape_name == "circle") ? SHAPE_CIRCLE : SHAPE_DIAMOND;
  marker_sub_ = nh_.subscribe(topic_name_, 1, &QRCodeMapWriter::markerCallback, this);

  initialized_ = true;
  name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized hector_geotiff MapWriter plugin %s.", name_.c_str());
}

void QRCodeMapWriter::draw(MapWriterInterface* interface)
{
  if (!initialized_) {
    return;
  }

  std::vector<StoredLabel> labels;
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    labels = stored_labels_;
  }

  for (std::size_t i = 0; i < labels.size(); ++i) {
    interface->drawObjectOfInterest(
      labels[i].point,
      labels[i].label,
      makeColor(labels[i].color_index),
      point_shape_
    );
  }
}

void QRCodeMapWriter::markerCallback(const visualization_msgs::MarkerArrayConstPtr& marker_array)
{
  boost::lock_guard<boost::mutex> lock(mutex_);

  for (std::size_t i = 0; i < marker_array->markers.size(); ++i) {
    const visualization_msgs::Marker& marker = marker_array->markers[i];

    if (marker.action != visualization_msgs::Marker::ADD) {
      continue;
    }
    if (marker.type != visualization_msgs::Marker::TEXT_VIEW_FACING) {
      continue;
    }
    if (marker.text.empty()) {
      continue;
    }

    const float x = static_cast<float>(marker.pose.position.x);
    const float y = static_cast<float>(marker.pose.position.y);
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }

    const Eigen::Vector2f point(x, y);
    if (isDuplicatePoint(point, stored_labels_)) {
      continue;
    }

    StoredLabel label;
    label.point = point;
    label.label = marker.text;
    label.color_index = stored_labels_.size();
    stored_labels_.push_back(label);

    ROS_INFO_NAMED(
      name_,
      "Registered persistent QR mark %s at (%.2f, %.2f).",
      label.label.c_str(),
      point.x(),
      point.y()
    );
  }
}

bool QRCodeMapWriter::isDuplicatePoint(
  const Eigen::Vector2f& point,
  const std::vector<StoredLabel>& existing_labels
) const
{
  for (std::size_t i = 0; i < existing_labels.size(); ++i) {
    const Eigen::Vector2f delta = existing_labels[i].point - point;
    if (delta.squaredNorm() <= min_separation_distance_squared_) {
      return true;
    }
  }

  return false;
}

MapWriterInterface::Color QRCodeMapWriter::makeColor(std::size_t index) const
{
  if (!use_palette_colors_) {
    return MapWriterInterface::Color(point_color_r_, point_color_g_, point_color_b_);
  }

  static const unsigned int palette[][3] = {
    {231, 111, 81},
    {42, 157, 143},
    {233, 196, 106},
    {69, 123, 157},
    {126, 87, 194},
    {244, 162, 97},
    {38, 70, 83},
    {230, 57, 70}
  };

  const std::size_t palette_size = sizeof(palette) / sizeof(palette[0]);
  const unsigned int* rgb = palette[index % palette_size];
  return MapWriterInterface::Color(rgb[0], rgb[1], rgb[2]);
}

} // namespace hector_geotiff_plugins

PLUGINLIB_EXPORT_CLASS(hector_geotiff_plugins::QRCodeMapWriter, hector_geotiff::MapWriterPluginInterface)
