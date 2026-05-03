/**
 * @file sensor_coverage_planner_ground.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"
#include "graph/graph.h"

namespace sensor_coverage_planner_3d_ns
{
bool PlannerParameters::ReadParameters(ros::NodeHandle& nh)
{
  sub_start_exploration_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_start_exploration_topic_", "/exploration_start");
  sub_state_estimation_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_state_estimation_topic_", "/state_estimation_at_scan");
  sub_registered_scan_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_registered_scan_topic_", "/registered_scan");
  sub_terrain_map_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_terrain_map_topic_", "/terrain_map");
  sub_terrain_map_ext_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_terrain_map_ext_topic_", "/terrain_map_ext");
  sub_coverage_boundary_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_coverage_boundary_topic_", "/coverage_boundary");
  sub_viewpoint_boundary_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_viewpoint_boundary_topic_", "/navigation_boundary");
  sub_nogo_boundary_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_nogo_boundary_topic_", "/nogo_boundary");
  sub_joystick_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_joystick_topic_", "/joy");
  sub_reset_waypoint_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_reset_waypoint_topic_", "/reset_waypoint");
  pub_exploration_finish_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_exploration_finish_topic_", "exploration_finish");
  pub_runtime_breakdown_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_runtime_breakdown_topic_", "runtime_breakdown");
  pub_runtime_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_runtime_topic_", "/runtime");
  pub_waypoint_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_waypoint_topic_", "/way_point");
  pub_momentum_activation_count_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_momentum_activation_count_topic_", "momentum_activation_count");
  pub_stuck_recovery_mode_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_stuck_recovery_mode_topic_", "/stuck_recovery_mode");

  // Bool
  kAutoStart = misc_utils_ns::getParam<bool>(nh, "kAutoStart", false);
  kRushHome = misc_utils_ns::getParam<bool>(nh, "kRushHome", false);
  kUseTerrainHeight = misc_utils_ns::getParam<bool>(nh, "kUseTerrainHeight", true);
  kCheckTerrainCollision = misc_utils_ns::getParam<bool>(nh, "kCheckTerrainCollision", true);
  kExtendWayPoint = misc_utils_ns::getParam<bool>(nh, "kExtendWayPoint", true);
  kUseLineOfSightLookAheadPoint = misc_utils_ns::getParam<bool>(nh, "kUseLineOfSightLookAheadPoint", true);
  kNoExplorationReturnHome = misc_utils_ns::getParam<bool>(nh, "kNoExplorationReturnHome", true);
  kUseMomentum = misc_utils_ns::getParam<bool>(nh, "kUseMomentum", false);
  kEnableBranchRecovery = misc_utils_ns::getParam<bool>(nh, "kEnableBranchRecovery", true);

  // Double
  kKeyposeCloudDwzFilterLeafSize = misc_utils_ns::getParam<double>(nh, "kKeyposeCloudDwzFilterLeafSize", 0.2);
  kRushHomeDist = misc_utils_ns::getParam<double>(nh, "kRushHomeDist", 10.0);
  kAtHomeDistThreshold = misc_utils_ns::getParam<double>(nh, "kAtHomeDistThreshold", 0.5);
  kTerrainCollisionThreshold = misc_utils_ns::getParam<double>(nh, "kTerrainCollisionThreshold", 0.5);
  kLookAheadDistance = misc_utils_ns::getParam<double>(nh, "kLookAheadDistance", 5.0);
  kLookAheadKeepMinDistance = misc_utils_ns::getParam<double>(nh, "kLookAheadKeepMinDistance", 0.8);
  kLookAheadSwitchScoreMargin = misc_utils_ns::getParam<double>(nh, "kLookAheadSwitchScoreMargin", 0.12);
  kExtendWayPointDistanceBig = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceBig", 8.0);
  kExtendWayPointDistanceSmall = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceSmall", 3.0);
  kBranchAnchorMinSeparation = misc_utils_ns::getParam<double>(nh, "kBranchAnchorMinSeparation", 1.0);
  kRecoveryAnchorReachedDist = misc_utils_ns::getParam<double>(nh, "kRecoveryAnchorReachedDist", 0.6);
  kRecoveryProgressMinDist = misc_utils_ns::getParam<double>(nh, "kRecoveryProgressMinDist", 0.15);
  kRecoveryWaypointMaxDistance = misc_utils_ns::getParam<double>(nh, "kRecoveryWaypointMaxDistance", 1.2);
  kRecoveryWaypointMinDot = misc_utils_ns::getParam<double>(nh, "kRecoveryWaypointMinDot", -0.35);
  kCompletedBranchAnchorActiveDist = misc_utils_ns::getParam<double>(nh, "kCompletedBranchAnchorActiveDist", 0.8);
  kCompletedBranchSameDirectionDotThr =
      misc_utils_ns::getParam<double>(nh, "kCompletedBranchSameDirectionDotThr", 0.82);

  // Int
  kReturnHomeCandidateCountThreshold =
      misc_utils_ns::getParam<int>(nh, "kReturnHomeCandidateCountThreshold", 8);
  kDirectionChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionChangeCounterThr", 4);
  kDirectionNoChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionNoChangeCounterThr", 5);
  kResetWaypointJoystickAxesID = misc_utils_ns::getParam<int>(nh, "kResetWaypointJoystickAxesID", 0);
  kBranchCandidateDegreeThreshold = misc_utils_ns::getParam<int>(nh, "kBranchCandidateDegreeThreshold", 3);
  kStuckCycleThreshold = misc_utils_ns::getParam<int>(nh, "kStuckCycleThreshold", 4);
  kRecoveryBreadcrumbLookback = misc_utils_ns::getParam<int>(nh, "kRecoveryBreadcrumbLookback", 12);

  return true;
}

void PlannerData::Initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
{
  keypose_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", kWorldFrameID);
  registered_scan_stack_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(nh, "registered_scan_stack", kWorldFrameID);
  registered_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "registered_cloud", kWorldFrameID);
  large_terrain_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud_large", kWorldFrameID);
  terrain_collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_collision_cloud", kWorldFrameID);
  terrain_ext_collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_ext_collision_cloud", kWorldFrameID);
  viewpoint_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "viewpoint_vis_cloud", kWorldFrameID);
  grid_world_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "grid_world_vis_cloud", kWorldFrameID);
  exploration_path_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "bspline_path_cloud", kWorldFrameID);

  selected_viewpoint_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "selected_viewpoint_vis_cloud", kWorldFrameID);
  exploring_cell_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "exploring_cell_vis_cloud", kWorldFrameID);
  collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "collision_cloud", kWorldFrameID);
  lookahead_point_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "lookahead_point_cloud", kWorldFrameID);
  keypose_graph_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "keypose_graph_cloud", kWorldFrameID);
  viewpoint_in_collision_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "viewpoint_in_collision_cloud_", kWorldFrameID);
  point_cloud_manager_neighbor_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "pointcloud_manager_cloud", kWorldFrameID);
  reordered_global_subspace_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "reordered_global_subspace_cloud", kWorldFrameID);

  planning_env_ = std::make_unique<planning_env_ns::PlanningEnv>(nh, nh_p);
  viewpoint_manager_ = std::make_shared<viewpoint_manager_ns::ViewPointManager>(nh_p);
  local_coverage_planner_ = std::make_unique<local_coverage_planner_ns::LocalCoveragePlanner>(nh_p);
  local_coverage_planner_->SetViewPointManager(viewpoint_manager_);
  keypose_graph_ = std::make_unique<keypose_graph_ns::KeyposeGraph>(nh_p);
  grid_world_ = std::make_unique<grid_world_ns::GridWorld>(nh_p);
  grid_world_->SetUseKeyposeGraph(true);
  visualizer_ = std::make_unique<tare_visualizer_ns::TAREVisualizer>(nh, nh_p);

  initial_position_.x() = 0.0;
  initial_position_.y() = 0.0;
  initial_position_.z() = 0.0;

  cur_keypose_node_ind_ = 0;

  keypose_graph_node_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "keypose_graph_node_marker", kWorldFrameID);
  keypose_graph_node_marker_->SetType(visualization_msgs::Marker::POINTS);
  keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  keypose_graph_edge_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "keypose_graph_edge_marker", kWorldFrameID);
  keypose_graph_edge_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

  nogo_boundary_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "nogo_boundary_marker", kWorldFrameID);
  nogo_boundary_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  nogo_boundary_marker_->SetScale(0.05, 0.0, 0.0);
  nogo_boundary_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  grid_world_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "grid_world_marker", kWorldFrameID);
  grid_world_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
  grid_world_marker_->SetScale(1.0, 1.0, 1.0);
  grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  robot_yaw_ = 0.0;
  lookahead_point_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_forward_ = true;

  Eigen::Vector3d viewpoint_resolution = viewpoint_manager_->GetResolution();
  double add_non_keypose_node_min_dist = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
  keypose_graph_->SetAddNonKeyposeNodeMinDist() = add_non_keypose_node_min_dist;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  last_robot_position_ = robot_position_;
}

SensorCoveragePlanner3D::SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
  : keypose_cloud_update_(false)
  , initialized_(false)
  , lookahead_point_update_(false)
  , relocation_(false)
  , start_exploration_(false)
  , exploration_finished_(false)
  , near_home_(false)
  , at_home_(false)
  , stopped_(false)
  , test_point_update_(false)
  , viewpoint_ind_update_(false)
  , step_(false)
  , use_momentum_(false)
  , recovery_mode_(false)
  , stuck_recovery_mode_(false)
  , progress_tracking_initialized_(false)
  , lookahead_point_in_line_of_sight_(true)
  , reset_waypoint_(false)
  , completed_branch_backtrack_requested_(false)
  , registered_cloud_count_(0)
  , keypose_count_(0)
  , direction_change_count_(0)
  , direction_no_change_count_(0)
  , momentum_activation_count_(0)
  , recovery_activation_count_(0)
  , return_home_candidate_count_(0)
  , recovery_target_visited_index_(-1)
  , recovery_anchor_stack_index_(-1)
  , recovery_source_visited_index_(-1)
  , stuck_cycle_count_(0)
  , reset_waypoint_joystick_axis_value_(-1.0)
{
  initialize(nh, nh_p);
  PrintExplorationStatus("Exploration Started", false);
}

bool SensorCoveragePlanner3D::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
{
  if (!pp_.ReadParameters(nh_p))
  {
    ROS_ERROR("Read parameters failed");
    return false;
  }

  pd_.Initialize(nh, nh_p);

  pd_.keypose_graph_->SetAllowVerticalEdge(false);

  lidar_model_ns::LiDARModel::setCloudDWZResol(pd_.planning_env_->GetPlannerCloudResolution());

  execution_timer_ = nh.createTimer(ros::Duration(1.0), &SensorCoveragePlanner3D::execute, this);

  exploration_start_sub_ =
      nh.subscribe(pp_.sub_start_exploration_topic_, 5, &SensorCoveragePlanner3D::ExplorationStartCallback, this);
  registered_scan_sub_ =
      nh.subscribe(pp_.sub_registered_scan_topic_, 5, &SensorCoveragePlanner3D::RegisteredScanCallback, this);
  terrain_map_sub_ = nh.subscribe(pp_.sub_terrain_map_topic_, 5, &SensorCoveragePlanner3D::TerrainMapCallback, this);
  terrain_map_ext_sub_ =
      nh.subscribe(pp_.sub_terrain_map_ext_topic_, 5, &SensorCoveragePlanner3D::TerrainMapExtCallback, this);
  state_estimation_sub_ =
      nh.subscribe(pp_.sub_state_estimation_topic_, 5, &SensorCoveragePlanner3D::StateEstimationCallback, this);
  coverage_boundary_sub_ =
      nh.subscribe(pp_.sub_coverage_boundary_topic_, 1, &SensorCoveragePlanner3D::CoverageBoundaryCallback, this);
  viewpoint_boundary_sub_ =
      nh.subscribe(pp_.sub_viewpoint_boundary_topic_, 1, &SensorCoveragePlanner3D::ViewPointBoundaryCallback, this);
  nogo_boundary_sub_ =
      nh.subscribe(pp_.sub_nogo_boundary_topic_, 1, &SensorCoveragePlanner3D::NogoBoundaryCallback, this);
  joystick_sub_ =
      nh.subscribe(pp_.sub_joystick_topic_, 1, &SensorCoveragePlanner3D::JoystickCallback, this);
  reset_waypoint_sub_ =
      nh.subscribe(pp_.sub_reset_waypoint_topic_, 1, &SensorCoveragePlanner3D::ResetWaypointCallback, this);

  global_path_full_publisher_ = nh.advertise<nav_msgs::Path>("global_path_full", 1);
  global_path_publisher_ = nh.advertise<nav_msgs::Path>("global_path", 1);
  old_global_path_publisher_ = nh.advertise<nav_msgs::Path>("old_global_path", 1);
  to_nearest_global_subspace_path_publisher_ = nh.advertise<nav_msgs::Path>("to_nearest_global_subspace_path", 1);
  local_tsp_path_publisher_ = nh.advertise<nav_msgs::Path>("local_path", 1);
  exploration_path_publisher_ = nh.advertise<nav_msgs::Path>("exploration_path", 1);
  waypoint_pub_ = nh.advertise<geometry_msgs::PointStamped>(pp_.pub_waypoint_topic_, 2);
  exploration_finish_pub_ = nh.advertise<std_msgs::Bool>(pp_.pub_exploration_finish_topic_, 2);
  runtime_breakdown_pub_ = nh.advertise<std_msgs::Int32MultiArray>(pp_.pub_runtime_breakdown_topic_, 2);
  runtime_pub_ = nh.advertise<std_msgs::Float32>(pp_.pub_runtime_topic_, 2);
  momentum_activation_count_pub_ = nh.advertise<std_msgs::Int32>(pp_.pub_momentum_activation_count_topic_, 2);
  stuck_recovery_mode_pub_ = nh.advertise<std_msgs::Bool>(pp_.pub_stuck_recovery_mode_topic_, 2);
  // Debug
  pointcloud_manager_neighbor_cells_origin_pub_ =
      nh.advertise<geometry_msgs::PointStamped>("pointcloud_manager_neighbor_cells_origin", 1);

  return true;
}

void SensorCoveragePlanner3D::ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg)
{
  if (start_msg->data)
  {
    start_exploration_ = true;
  }
}

void SensorCoveragePlanner3D::StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg)
{
  pd_.robot_position_ = state_estimation_msg->pose.pose.position;
  // Todo: use a boolean
  if (std::abs(pd_.initial_position_.x()) < 0.01 && std::abs(pd_.initial_position_.y()) < 0.01 &&
      std::abs(pd_.initial_position_.z()) < 0.01)
  {
    pd_.initial_position_.x() = pd_.robot_position_.x;
    pd_.initial_position_.y() = pd_.robot_position_.y;
    pd_.initial_position_.z() = pd_.robot_position_.z;
  }
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_quat = state_estimation_msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

  pd_.robot_yaw_ = yaw;

  if (state_estimation_msg->twist.twist.linear.x > 0.4)
  {
    pd_.moving_forward_ = true;
  }
  else if (state_estimation_msg->twist.twist.linear.x < -0.4)
  {
    pd_.moving_forward_ = false;
  }
  initialized_ = true;
}

void SensorCoveragePlanner3D::RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_scan_msg)
{
  if (!initialized_)
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_tmp(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*registered_scan_msg, *registered_scan_tmp);
  if (registered_scan_tmp->points.empty())
  {
    return;
  }
  *(pd_.registered_scan_stack_->cloud_) += *(registered_scan_tmp);
  pointcloud_downsizer_.Downsize(registered_scan_tmp, pp_.kKeyposeCloudDwzFilterLeafSize,
                                 pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);
  pd_.registered_cloud_->cloud_->clear();
  pcl::copyPointCloud(*registered_scan_tmp, *(pd_.registered_cloud_->cloud_));

  pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_);
  pd_.planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(pd_.registered_cloud_->cloud_);

  registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;
  if (registered_cloud_count_ == 0)
  {
    // initialized_ = true;
    pd_.keypose_.pose.pose.position = pd_.robot_position_;
    pd_.keypose_.pose.covariance[0] = keypose_count_++;
    pd_.cur_keypose_node_ind_ = pd_.keypose_graph_->AddKeyposeNode(pd_.keypose_, *(pd_.planning_env_));

    pointcloud_downsizer_.Downsize(pd_.registered_scan_stack_->cloud_, pp_.kKeyposeCloudDwzFilterLeafSize,
                                   pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);

    pd_.keypose_cloud_->cloud_->clear();
    pcl::copyPointCloud(*(pd_.registered_scan_stack_->cloud_), *(pd_.keypose_cloud_->cloud_));
    // pd_.keypose_cloud_->Publish();
    pd_.registered_scan_stack_->cloud_->clear();
    keypose_cloud_update_ = true;
  }
}

void SensorCoveragePlanner3D::TerrainMapCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_msg)
{
  if (pp_.kCheckTerrainCollision)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);
    pd_.terrain_collision_cloud_->cloud_->clear();
    for (auto& point : terrain_map_tmp->points)
    {
      if (point.intensity > pp_.kTerrainCollisionThreshold)
      {
        pd_.terrain_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::TerrainMapExtCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_ext_msg)
{
  if (pp_.kUseTerrainHeight)
  {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(pd_.large_terrain_cloud_->cloud_));
  }
  if (pp_.kCheckTerrainCollision)
  {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(pd_.large_terrain_cloud_->cloud_));
    pd_.terrain_ext_collision_cloud_->cloud_->clear();
    for (auto& point : pd_.large_terrain_cloud_->cloud_->points)
    {
      if (point.intensity > pp_.kTerrainCollisionThreshold)
      {
        pd_.terrain_ext_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void SensorCoveragePlanner3D::CoverageBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  pd_.planning_env_->UpdateCoverageBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::ViewPointBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  pd_.viewpoint_manager_->UpdateViewPointBoundary((*polygon_msg).polygon);
}

void SensorCoveragePlanner3D::NogoBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  if (polygon_msg->polygon.points.empty())
  {
    return;
  }
  double polygon_id = polygon_msg->polygon.points[0].z;
  int polygon_point_size = polygon_msg->polygon.points.size();
  std::vector<geometry_msgs::Polygon> nogo_boundary;
  geometry_msgs::Polygon polygon;
  for (int i = 0; i < polygon_point_size; i++)
  {
    if (polygon_msg->polygon.points[i].z == polygon_id)
    {
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
    else
    {
      nogo_boundary.push_back(polygon);
      polygon.points.clear();
      polygon_id = polygon_msg->polygon.points[i].z;
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
  }
  nogo_boundary.push_back(polygon);
  pd_.viewpoint_manager_->UpdateNogoBoundary(nogo_boundary);

  geometry_msgs::Point point;
  for (int i = 0; i < nogo_boundary.size(); i++)
  {
    for (int j = 0; j < nogo_boundary[i].points.size() - 1; j++)
    {
      point.x = nogo_boundary[i].points[j].x;
      point.y = nogo_boundary[i].points[j].y;
      point.z = nogo_boundary[i].points[j].z;
      pd_.nogo_boundary_marker_->marker_.points.push_back(point);
      point.x = nogo_boundary[i].points[j + 1].x;
      point.y = nogo_boundary[i].points[j + 1].y;
      point.z = nogo_boundary[i].points[j + 1].z;
      pd_.nogo_boundary_marker_->marker_.points.push_back(point);
    }
    point.x = nogo_boundary[i].points.back().x;
    point.y = nogo_boundary[i].points.back().y;
    point.z = nogo_boundary[i].points.back().z;
    pd_.nogo_boundary_marker_->marker_.points.push_back(point);
    point.x = nogo_boundary[i].points.front().x;
    point.y = nogo_boundary[i].points.front().y;
    point.z = nogo_boundary[i].points.front().z;
    pd_.nogo_boundary_marker_->marker_.points.push_back(point);
  }
  pd_.nogo_boundary_marker_->Publish();
}

void SensorCoveragePlanner3D::JoystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg){
   if (pp_.kResetWaypointJoystickAxesID >= 0 && pp_.kResetWaypointJoystickAxesID < joy_msg->axes.size()) {
    if (reset_waypoint_joystick_axis_value_ > -0.1 && joy_msg->axes[pp_.kResetWaypointJoystickAxesID] < -0.1) {
      reset_waypoint_ = true;
      // Set waypoint to the current robot position to stop the robot in place
      geometry_msgs::PointStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.header.stamp = ros::Time::now();
      waypoint.point.x = pd_.robot_position_.x;
      waypoint.point.y = pd_.robot_position_.y;
      waypoint.point.z = pd_.robot_position_.z;
      waypoint_pub_.publish(waypoint);
      std::cout << "reset waypoint" << std::endl;
    }
    reset_waypoint_joystick_axis_value_ = joy_msg->axes[pp_.kResetWaypointJoystickAxesID];
  }
  
}

void SensorCoveragePlanner3D::ResetWaypointCallback(const std_msgs::Empty::ConstPtr& empty_msg){
  reset_waypoint_ = true;
  // Set waypoint to the current robot position to stop the robot in place
  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = pd_.robot_position_.x;
  waypoint.point.y = pd_.robot_position_.y;
  waypoint.point.z = pd_.robot_position_.z;
  waypoint_pub_.publish(waypoint);
  std::cout << "reset waypoint" << std::endl;
}

void SensorCoveragePlanner3D::SendInitialWaypoint()
{
  // send waypoint ahead
  double lx = 12.0;
  double ly = 0.0;
  double dx = cos(pd_.robot_yaw_) * lx - sin(pd_.robot_yaw_) * ly;
  double dy = sin(pd_.robot_yaw_) * lx + cos(pd_.robot_yaw_) * ly;

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = pd_.robot_position_.x + dx;
  waypoint.point.y = pd_.robot_position_.y + dy;
  waypoint.point.z = pd_.robot_position_.z;
  waypoint_pub_.publish(waypoint);
}

void SensorCoveragePlanner3D::UpdateKeyposeGraph()
{
  misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
  update_keypose_graph_timer.Start();

  pd_.keypose_graph_->GetMarker(pd_.keypose_graph_node_marker_->marker_, pd_.keypose_graph_edge_marker_->marker_);
  // pd_.keypose_graph_node_marker_->Publish();
  pd_.keypose_graph_edge_marker_->Publish();
  pd_.keypose_graph_vis_cloud_->cloud_->clear();
  pd_.keypose_graph_->CheckLocalCollision(pd_.robot_position_, pd_.viewpoint_manager_);
  pd_.keypose_graph_->CheckConnectivity(pd_.robot_position_);
  pd_.keypose_graph_->GetVisualizationCloud(pd_.keypose_graph_vis_cloud_->cloud_);
  pd_.keypose_graph_vis_cloud_->Publish();

  update_keypose_graph_timer.Stop(false);
}

int SensorCoveragePlanner3D::UpdateViewPoints()
{
  misc_utils_ns::Timer collision_cloud_timer("update collision cloud");
  collision_cloud_timer.Start();
  pd_.collision_cloud_->cloud_ = pd_.planning_env_->GetCollisionCloud();
  collision_cloud_timer.Stop(false);

  misc_utils_ns::Timer viewpoint_manager_update_timer("update viewpoint manager");
  viewpoint_manager_update_timer.Start();
  if (pp_.kUseTerrainHeight)
  {
    pd_.viewpoint_manager_->SetViewPointHeightWithTerrain(pd_.large_terrain_cloud_->cloud_);
  }
  if (pp_.kCheckTerrainCollision)
  {
    *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_collision_cloud_->cloud_);
    *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_ext_collision_cloud_->cloud_);
  }
  pd_.viewpoint_manager_->CheckViewPointCollision(pd_.collision_cloud_->cloud_);
  pd_.viewpoint_manager_->CheckViewPointLineOfSight();
  pd_.viewpoint_manager_->CheckViewPointConnectivity();
  int viewpoint_candidate_count = pd_.viewpoint_manager_->GetViewPointCandidate();

  UpdateVisitedPositions();
  pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.visited_positions_);
  pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.grid_world_);

  // For visualization
  pd_.collision_cloud_->Publish();
  // pd_.collision_grid_cloud_->Publish();
  pd_.viewpoint_manager_->GetCollisionViewPointVisCloud(pd_.viewpoint_in_collision_cloud_->cloud_);
  pd_.viewpoint_in_collision_cloud_->Publish();

  viewpoint_manager_update_timer.Stop(false);
  return viewpoint_candidate_count;
}

void SensorCoveragePlanner3D::UpdateViewPointCoverage()
{
  // Update viewpoint coverage
  misc_utils_ns::Timer update_coverage_timer("update viewpoint coverage");
  update_coverage_timer.Start();
  pd_.viewpoint_manager_->UpdateViewPointCoverage<PlannerCloudPointType>(pd_.planning_env_->GetDiffCloud());
  pd_.viewpoint_manager_->UpdateRolledOverViewPointCoverage<PlannerCloudPointType>(
      pd_.planning_env_->GetStackedCloud());
  // Update robot coverage
  pd_.robot_viewpoint_.ResetCoverage();
  geometry_msgs::Pose robot_pose;
  robot_pose.position = pd_.robot_position_;
  pd_.robot_viewpoint_.setPose(robot_pose);
  UpdateRobotViewPointCoverage();
  update_coverage_timer.Stop(false);
}

void SensorCoveragePlanner3D::UpdateRobotViewPointCoverage()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pd_.planning_env_->GetCollisionCloud();
  for (const auto& point : cloud->points)
  {
    if (pd_.viewpoint_manager_->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z)))
    {
      pd_.robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
    }
  }
}

void SensorCoveragePlanner3D::UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Update covered area
  misc_utils_ns::Timer update_coverage_area_timer("update covered area");
  update_coverage_area_timer.Start();
  pd_.planning_env_->UpdateCoveredArea(pd_.robot_viewpoint_, pd_.viewpoint_manager_);
  update_coverage_area_timer.Stop(false);
  misc_utils_ns::Timer get_uncovered_area_timer("get uncovered area");
  get_uncovered_area_timer.Start();
  pd_.planning_env_->GetUncoveredArea(pd_.viewpoint_manager_, uncovered_point_num, uncovered_frontier_point_num);
  get_uncovered_area_timer.Stop(false);
  pd_.planning_env_->PublishUncoveredCloud();
  pd_.planning_env_->PublishUncoveredFrontierCloud();
}

void SensorCoveragePlanner3D::UpdateVisitedPositions()
{
  Eigen::Vector3d robot_current_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  bool existing = false;
  for (int i = 0; i < pd_.visited_positions_.size(); i++)
  {
    // TODO: parameterize this
    if ((robot_current_position - pd_.visited_positions_[i]).norm() < 1)
    {
      existing = true;
      break;
    }
  }
  if (!existing)
  {
    pd_.visited_positions_.push_back(robot_current_position);
  }
}

int SensorCoveragePlanner3D::GetNearestVisitedPositionIndex(const Eigen::Vector3d& position, double max_distance) const
{
  int nearest_index = -1;
  double nearest_distance = max_distance;
  for (int i = static_cast<int>(pd_.visited_positions_.size()) - 1; i >= 0; --i)
  {
    double distance = (position - pd_.visited_positions_[i]).norm();
    if (distance <= nearest_distance)
    {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  return nearest_index;
}

int SensorCoveragePlanner3D::GetActiveBranchAnchorStackIndex(const Eigen::Vector3d& position, double max_distance) const
{
  if (branch_anchors_.empty())
  {
    return -1;
  }

  int best_stack_index = -1;
  double best_distance = max_distance;
  for (int i = static_cast<int>(branch_anchors_.size()) - 1; i >= 0; --i)
  {
    int visited_index = branch_anchors_[i].visited_index_;
    if (visited_index < 0 || visited_index >= pd_.visited_positions_.size())
    {
      continue;
    }

    double distance = (position - pd_.visited_positions_[visited_index]).norm();
    if (distance <= best_distance)
    {
      best_distance = distance;
      best_stack_index = i;
    }
  }

  return best_stack_index;
}

void SensorCoveragePlanner3D::UpdateBranchAnchors()
{
  if (!pp_.kEnableBranchRecovery || recovery_mode_ || pd_.visited_positions_.empty())
  {
    return;
  }

  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  int current_visited_index = GetNearestVisitedPositionIndex(robot_position, 1.5);
  if (current_visited_index < 0)
  {
    current_visited_index = static_cast<int>(pd_.visited_positions_.size()) - 1;
  }

  while (!branch_anchor_indices_.empty() && branch_anchor_indices_.back() > current_visited_index)
  {
    branch_anchor_indices_.pop_back();
  }
  while (!branch_anchors_.empty() && branch_anchors_.back().visited_index_ > current_visited_index)
  {
    branch_anchors_.pop_back();
  }

  int candidate_degree = pd_.viewpoint_manager_->GetCandidateViewPointNeighborCount(robot_position);
  if (candidate_degree < pp_.kBranchCandidateDegreeThreshold)
  {
    return;
  }

  if (!branch_anchor_indices_.empty())
  {
    int last_anchor_index = branch_anchor_indices_.back();
    if (current_visited_index <= last_anchor_index)
    {
      return;
    }

    if ((pd_.visited_positions_[current_visited_index] - pd_.visited_positions_[last_anchor_index]).norm() <
        pp_.kBranchAnchorMinSeparation)
    {
      return;
    }
  }

  branch_anchor_indices_.push_back(current_visited_index);
  BranchAnchorState branch_anchor_state;
  branch_anchor_state.visited_index_ = current_visited_index;
  branch_anchors_.push_back(branch_anchor_state);
  ROS_INFO_STREAM("Recorded branch anchor at breadcrumb " << current_visited_index
                                                          << ", candidate degree: " << candidate_degree);
}

bool SensorCoveragePlanner3D::StartRecoveryToLatestAnchor(const std::string& reason)
{
  if (!pp_.kEnableBranchRecovery || branch_anchor_indices_.empty())
  {
    return false;
  }

  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  int current_visited_index = GetNearestVisitedPositionIndex(robot_position, 1.5);
  if (current_visited_index < 0)
  {
    current_visited_index = static_cast<int>(pd_.visited_positions_.size()) - 1;
  }

  for (int i = static_cast<int>(branch_anchor_indices_.size()) - 1; i >= 0; --i)
  {
    int anchor_index = branch_anchor_indices_[i];
    if (anchor_index < 0 || anchor_index >= pd_.visited_positions_.size() || anchor_index >= current_visited_index)
    {
      continue;
    }

    if ((pd_.visited_positions_[anchor_index] - robot_position).norm() <= pp_.kRecoveryAnchorReachedDist)
    {
      continue;
    }

    recovery_mode_ = true;
    stuck_recovery_mode_ = (reason == "robot progress stalled");
    recovery_target_visited_index_ = anchor_index;
    recovery_anchor_stack_index_ = i;
    recovery_source_visited_index_ = current_visited_index;
    completed_branch_backtrack_requested_ = (reason == "completed current dead-end branch");
    recovery_activation_count_++;
    stuck_cycle_count_ = 0;
    progress_tracking_initialized_ = true;
    progress_reference_position_ = robot_position;
    ROS_WARN_STREAM("Entering recovery mode toward breadcrumb " << anchor_index << ": " << reason
                    << ", stuck_recovery=" << (stuck_recovery_mode_ ? "true" : "false"));
    return true;
  }

  return false;
}

void SensorCoveragePlanner3D::ExitRecoveryMode(bool reached_anchor)
{
  if (reached_anchor && completed_branch_backtrack_requested_)
  {
    RememberCompletedBranchDirection();
  }

  if (reached_anchor)
  {
    while (!branch_anchor_indices_.empty() && branch_anchor_indices_.back() > recovery_target_visited_index_)
    {
      branch_anchor_indices_.pop_back();
    }
    while (!branch_anchors_.empty() && branch_anchors_.back().visited_index_ > recovery_target_visited_index_)
    {
      branch_anchors_.pop_back();
    }
  }

  recovery_mode_ = false;
  stuck_recovery_mode_ = false;
  recovery_target_visited_index_ = -1;
  recovery_anchor_stack_index_ = -1;
  recovery_source_visited_index_ = -1;
  completed_branch_backtrack_requested_ = false;
  stuck_cycle_count_ = 0;
  progress_tracking_initialized_ = true;
  progress_reference_position_ =
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
}

void SensorCoveragePlanner3D::RememberCompletedBranchDirection()
{
  if (recovery_anchor_stack_index_ < 0 || recovery_anchor_stack_index_ >= branch_anchors_.size())
  {
    return;
  }
  if (recovery_source_visited_index_ < 0 || recovery_source_visited_index_ >= pd_.visited_positions_.size())
  {
    return;
  }

  int anchor_visited_index = branch_anchors_[recovery_anchor_stack_index_].visited_index_;
  if (anchor_visited_index < 0 || anchor_visited_index >= pd_.visited_positions_.size())
  {
    return;
  }

  Eigen::Vector2d branch_direction =
      (pd_.visited_positions_[recovery_source_visited_index_] - pd_.visited_positions_[anchor_visited_index]).head<2>();
  double norm = branch_direction.norm();
  if (norm < 0.2)
  {
    return;
  }
  branch_direction /= norm;

  std::vector<Eigen::Vector2d>& completed_directions =
      branch_anchors_[recovery_anchor_stack_index_].completed_branch_directions_;
  for (const auto& existing_direction : completed_directions)
  {
    if (existing_direction.dot(branch_direction) >= pp_.kCompletedBranchSameDirectionDotThr)
    {
      return;
    }
  }

  completed_directions.push_back(branch_direction);
  ROS_INFO_STREAM("Marked completed branch direction at anchor breadcrumb " << anchor_visited_index
                  << ", total completed directions: " << completed_directions.size());
}

void SensorCoveragePlanner3D::UpdateProgressState(const Eigen::Vector3d& target_position, bool target_valid)
{
  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  if (!progress_tracking_initialized_)
  {
    progress_reference_position_ = robot_position;
    progress_tracking_initialized_ = true;
    stuck_cycle_count_ = 0;
    return;
  }

  if ((robot_position - progress_reference_position_).norm() > pp_.kRecoveryProgressMinDist)
  {
    progress_reference_position_ = robot_position;
    stuck_cycle_count_ = 0;
    return;
  }

  if (!target_valid)
  {
    return;
  }

  if ((target_position - robot_position).norm() <= pp_.kRecoveryAnchorReachedDist)
  {
    progress_reference_position_ = robot_position;
    stuck_cycle_count_ = 0;
    return;
  }

  stuck_cycle_count_++;
}

bool SensorCoveragePlanner3D::DirectionMatchesCompletedBranch(const BranchAnchorState& anchor_state,
                                                              const Eigen::Vector3d& candidate_position) const
{
  if (anchor_state.completed_branch_directions_.empty())
  {
    return false;
  }
  if (anchor_state.visited_index_ < 0 || anchor_state.visited_index_ >= pd_.visited_positions_.size())
  {
    return false;
  }

  Eigen::Vector2d candidate_direction =
      (candidate_position - pd_.visited_positions_[anchor_state.visited_index_]).head<2>();
  double norm = candidate_direction.norm();
  if (norm < 0.2)
  {
    return false;
  }
  candidate_direction /= norm;

  for (const auto& completed_direction : anchor_state.completed_branch_directions_)
  {
    if (completed_direction.dot(candidate_direction) >= pp_.kCompletedBranchSameDirectionDotThr)
    {
      return true;
    }
  }

  return false;
}

bool SensorCoveragePlanner3D::SelectRecoveryWaypointFromPath(const nav_msgs::Path& path, Eigen::Vector3d& waypoint)
{
  if (path.poses.size() < 2)
  {
    return false;
  }

  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  Eigen::Vector2d forward_dir(std::cos(pd_.robot_yaw_), std::sin(pd_.robot_yaw_));

  int best_index = -1;
  double best_score = -DBL_MAX;
  int fallback_index = -1;
  double fallback_score = -DBL_MAX;

  for (int i = 1; i < path.poses.size(); ++i)
  {
    Eigen::Vector3d candidate_point(path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                                    path.poses[i].pose.position.z);
    Eigen::Vector3d diff = candidate_point - robot_position;
    diff.z() = 0.0;
    double dist = diff.norm();
    if (dist < 0.05)
    {
      continue;
    }

    Eigen::Vector2d dir = diff.head<2>().normalized();
    double dot = forward_dir.dot(dir);
    if (dot > fallback_score)
    {
      fallback_score = dot;
      fallback_index = i;
    }

    if (dist <= pp_.kRecoveryWaypointMaxDistance && dot >= pp_.kRecoveryWaypointMinDot)
    {
      double score = dist + 0.2 * dot;
      if (score > best_score)
      {
        best_score = score;
        best_index = i;
      }
    }
  }

  if (best_index < 0)
  {
    best_index = fallback_index >= 0 ? fallback_index : 1;
  }

  waypoint.x() = path.poses[best_index].pose.position.x;
  waypoint.y() = path.poses[best_index].pose.position.y;
  waypoint.z() = path.poses[best_index].pose.position.z;
  return true;
}

bool SensorCoveragePlanner3D::UpdateRecoveryWaypoint()
{
  if (!recovery_mode_ || recovery_target_visited_index_ < 0 ||
      recovery_target_visited_index_ >= pd_.visited_positions_.size())
  {
    return false;
  }

  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  Eigen::Vector3d anchor_position = pd_.visited_positions_[recovery_target_visited_index_];
  if ((anchor_position - robot_position).norm() <= pp_.kRecoveryAnchorReachedDist)
  {
    ExitRecoveryMode(true);
    return false;
  }

  int current_visited_index = GetNearestVisitedPositionIndex(robot_position, 1.5);
  if (current_visited_index < 0)
  {
    current_visited_index = static_cast<int>(pd_.visited_positions_.size()) - 1;
  }

  if (current_visited_index <= recovery_target_visited_index_ + 1)
  {
    ExitRecoveryMode(true);
    return false;
  }

  int search_begin = std::max(recovery_target_visited_index_, current_visited_index - pp_.kRecoveryBreadcrumbLookback);
  nav_msgs::Path recovery_path;
  bool found_path = false;
  for (int breadcrumb_index = search_begin; breadcrumb_index < current_visited_index; ++breadcrumb_index)
  {
    const Eigen::Vector3d& breadcrumb_position = pd_.visited_positions_[breadcrumb_index];
    if (!pd_.viewpoint_manager_->InLocalPlanningHorizon(breadcrumb_position))
    {
      continue;
    }

    nav_msgs::Path candidate_path =
        pd_.viewpoint_manager_->GetViewPointShortestPath(robot_position, breadcrumb_position);
    if (candidate_path.poses.size() >= 2)
    {
      recovery_path = candidate_path;
      found_path = true;
      break;
    }
  }

  if (!found_path)
  {
    ROS_WARN_THROTTLE(2.0, "Recovery mode active, but no forward breadcrumb path is currently available");
    return false;
  }

  return SelectRecoveryWaypointFromPath(recovery_path, recovery_waypoint_);
}

void SensorCoveragePlanner3D::UpdateGlobalRepresentation()
{
  pd_.local_coverage_planner_->SetRobotPosition(
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z));
  bool viewpoint_rollover = pd_.viewpoint_manager_->UpdateRobotPosition(
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z));
  if (!pd_.grid_world_->Initialized() || viewpoint_rollover)
  {
    pd_.grid_world_->UpdateNeighborCells(pd_.robot_position_);
  }

  pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_);
  pd_.planning_env_->GetVisualizationPointCloud(pd_.point_cloud_manager_neighbor_cloud_->cloud_);
  pd_.point_cloud_manager_neighbor_cloud_->Publish();

  // DEBUG
  Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
      pd_.planning_env_->GetPointCloudManagerNeighborCellsOrigin();
  geometry_msgs::PointStamped pointcloud_manager_neighbor_cells_origin_point;
  pointcloud_manager_neighbor_cells_origin_point.header.frame_id = "map";
  pointcloud_manager_neighbor_cells_origin_point.header.stamp = ros::Time::now();
  pointcloud_manager_neighbor_cells_origin_point.point.x = pointcloud_manager_neighbor_cells_origin.x();
  pointcloud_manager_neighbor_cells_origin_point.point.y = pointcloud_manager_neighbor_cells_origin.y();
  pointcloud_manager_neighbor_cells_origin_point.point.z = pointcloud_manager_neighbor_cells_origin.z();
  pointcloud_manager_neighbor_cells_origin_pub_.publish(pointcloud_manager_neighbor_cells_origin_point);

  if (exploration_finished_ && pp_.kNoExplorationReturnHome)
  {
    pd_.planning_env_->SetUseFrontier(false);
  }
  pd_.planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(pd_.keypose_cloud_->cloud_);

  int closest_node_ind = pd_.keypose_graph_->GetClosestNodeInd(pd_.robot_position_);
  geometry_msgs::Point closest_node_position = pd_.keypose_graph_->GetClosestNodePosition(pd_.robot_position_);
  pd_.grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind);
  pd_.grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position);

  pd_.grid_world_->UpdateRobotPosition(pd_.robot_position_);
  if (!pd_.grid_world_->HomeSet())
  {
    pd_.grid_world_->SetHomePosition(pd_.initial_position_);
  }
}

void SensorCoveragePlanner3D::GlobalPlanning(std::vector<int>& global_cell_tsp_order,
                                             exploration_path_ns::ExplorationPath& global_path)
{
  misc_utils_ns::Timer global_tsp_timer("Global planning");
  global_tsp_timer.Start();

  pd_.grid_world_->UpdateCellStatus(pd_.viewpoint_manager_);
  pd_.grid_world_->UpdateCellKeyposeGraphNodes(pd_.keypose_graph_);
  pd_.grid_world_->AddPathsInBetweenCells(pd_.viewpoint_manager_, pd_.keypose_graph_);

  pd_.viewpoint_manager_->UpdateCandidateViewPointCellStatus(pd_.grid_world_);

  global_path = pd_.grid_world_->SolveGlobalTSP(pd_.viewpoint_manager_, global_cell_tsp_order, pd_.keypose_graph_);

  global_tsp_timer.Stop(false);
  global_planning_runtime_ = global_tsp_timer.GetDuration("ms");
}

void SensorCoveragePlanner3D::PublishGlobalPlanningVisualization(
    const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path)
{
  nav_msgs::Path global_path_full = global_path.GetPath();
  global_path_full.header.frame_id = "map";
  global_path_full.header.stamp = ros::Time::now();
  global_path_full_publisher_.publish(global_path_full);
  // Get the part that connects with the local path

  int start_index = 0;
  for (int i = 0; i < global_path.nodes_.size(); i++)
  {
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !pd_.viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
    {
      break;
    }
    start_index = i;
  }

  int end_index = global_path.nodes_.size() - 1;
  for (int i = global_path.nodes_.size() - 1; i >= 0; i--)
  {
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !pd_.viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
    {
      break;
    }
    end_index = i;
  }

  nav_msgs::Path global_path_trim;
  if (local_path.nodes_.size() >= 2)
  {
    geometry_msgs::PoseStamped first_pose;
    first_pose.pose.position.x = local_path.nodes_.front().position_.x();
    first_pose.pose.position.y = local_path.nodes_.front().position_.y();
    first_pose.pose.position.z = local_path.nodes_.front().position_.z();
    global_path_trim.poses.push_back(first_pose);
  }

  for (int i = start_index; i <= end_index; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = global_path.nodes_[i].position_.x();
    pose.pose.position.y = global_path.nodes_[i].position_.y();
    pose.pose.position.z = global_path.nodes_[i].position_.z();
    global_path_trim.poses.push_back(pose);
  }
  if (local_path.nodes_.size() >= 2)
  {
    geometry_msgs::PoseStamped last_pose;
    last_pose.pose.position.x = local_path.nodes_.back().position_.x();
    last_pose.pose.position.y = local_path.nodes_.back().position_.y();
    last_pose.pose.position.z = local_path.nodes_.back().position_.z();
    global_path_trim.poses.push_back(last_pose);
  }
  global_path_trim.header.frame_id = "map";
  global_path_trim.header.stamp = ros::Time::now();
  global_path_publisher_.publish(global_path_trim);

  pd_.grid_world_->GetVisualizationCloud(pd_.grid_world_vis_cloud_->cloud_);
  pd_.grid_world_vis_cloud_->Publish();
  pd_.grid_world_->GetMarker(pd_.grid_world_marker_->marker_);
  pd_.grid_world_marker_->Publish();
  nav_msgs::Path full_path = pd_.exploration_path_.GetPath();
  full_path.header.frame_id = "map";
  full_path.header.stamp = ros::Time::now();
  // exploration_path_publisher_.publish(full_path);
  pd_.exploration_path_.GetVisualizationCloud(pd_.exploration_path_cloud_->cloud_);
  pd_.exploration_path_cloud_->Publish();
  // pd_.planning_env_->PublishStackedCloud();
}

void SensorCoveragePlanner3D::LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                                            const exploration_path_ns::ExplorationPath& global_path,
                                            exploration_path_ns::ExplorationPath& local_path)
{
  misc_utils_ns::Timer local_tsp_timer("Local planning");
  local_tsp_timer.Start();
  if (lookahead_point_update_)
  {
    pd_.local_coverage_planner_->SetLookAheadPoint(pd_.lookahead_point_);
  }
  local_path = pd_.local_coverage_planner_->SolveLocalCoverageProblem(global_path, uncovered_point_num,
                                                                      uncovered_frontier_point_num);
  local_tsp_timer.Stop(false);
}

void SensorCoveragePlanner3D::PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path)
{
  pd_.viewpoint_manager_->GetVisualizationCloud(pd_.viewpoint_vis_cloud_->cloud_);
  pd_.viewpoint_vis_cloud_->Publish();
  pd_.lookahead_point_cloud_->Publish();
  nav_msgs::Path local_tsp_path = local_path.GetPath();
  local_tsp_path.header.frame_id = "map";
  local_tsp_path.header.stamp = ros::Time::now();
  local_tsp_path_publisher_.publish(local_tsp_path);
  pd_.local_coverage_planner_->GetSelectedViewPointVisCloud(pd_.selected_viewpoint_vis_cloud_->cloud_);
  pd_.selected_viewpoint_vis_cloud_->Publish();

  // Visualize local planning horizon box
}

exploration_path_ns::ExplorationPath SensorCoveragePlanner3D::ConcatenateGlobalLocalPath(
    const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path)
{
  exploration_path_ns::ExplorationPath full_path;
  if (exploration_finished_ && near_home_ && pp_.kRushHome)
  {
    exploration_path_ns::Node node;
    node.position_.x() = pd_.robot_position_.x;
    node.position_.y() = pd_.robot_position_.y;
    node.position_.z() = pd_.robot_position_.z;
    node.type_ = exploration_path_ns::NodeType::ROBOT;
    full_path.nodes_.push_back(node);
    node.position_ = pd_.initial_position_;
    node.type_ = exploration_path_ns::NodeType::HOME;
    full_path.nodes_.push_back(node);
    return full_path;
  }

  double global_path_length = global_path.GetLength();
  double local_path_length = local_path.GetLength();
  if (global_path_length < 3 && local_path_length < 5)
  {
    return full_path;
  }
  else
  {
    full_path = local_path;
    if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
        local_path.nodes_.back().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START)
    {
      full_path.Reverse();
    }
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START &&
             local_path.nodes_.back() == local_path.nodes_.front())
    {
      full_path.nodes_.back().type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
    }
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
             local_path.nodes_.back() == local_path.nodes_.front())
    {
      full_path.nodes_.front().type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
    }
  }

  return full_path;
}

bool SensorCoveragePlanner3D::GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                                                const exploration_path_ns::ExplorationPath& global_path,
                                                Eigen::Vector3d& lookahead_point)
{
  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);

  // Determine which direction to follow on the global path
  double dist_from_start = 0.0;
  for (int i = 1; i < global_path.nodes_.size(); i++)
  {
    dist_from_start += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
    {
      break;
    }
  }

  double dist_from_end = 0.0;
  for (int i = global_path.nodes_.size() - 2; i > 0; i--)
  {
    dist_from_end += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
    {
      break;
    }
  }

  bool local_path_too_short = true;
  for (int i = 0; i < local_path.nodes_.size(); i++)
  {
    double dist_to_robot = (robot_position - local_path.nodes_[i].position_).norm();
    if (dist_to_robot > pp_.kLookAheadDistance / 5)
    {
      local_path_too_short = false;
      break;
    }
  }
  if (local_path.GetNodeNum() < 1 || local_path_too_short)
  {
    if (dist_from_start < dist_from_end)
    {
      double dist_from_robot = 0.0;
      for (int i = 1; i < global_path.nodes_.size(); i++)
      {
        dist_from_robot += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
        if (dist_from_robot > pp_.kLookAheadDistance / 2)
        {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    }
    else
    {
      double dist_from_robot = 0.0;
      for (int i = global_path.nodes_.size() - 2; i > 0; i--)
      {
        dist_from_robot += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
        if (dist_from_robot > pp_.kLookAheadDistance / 2)
        {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    }
    return false;
  }

  bool has_lookahead = false;
  bool dir = true;
  int robot_i = 0;
  int lookahead_i = 0;
  for (int i = 0; i < local_path.nodes_.size(); i++)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT)
    {
      robot_i = i;
    }
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOOKAHEAD_POINT)
    {
      has_lookahead = true;
      lookahead_i = i;
    }
  }

  if(reset_waypoint_){
    has_lookahead = false;   
  }

  int forward_viewpoint_count = 0;
  int backward_viewpoint_count = 0;

  bool local_loop = false;
  if (local_path.nodes_.front() == local_path.nodes_.back() &&
      local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT)
  {
    local_loop = true;
  }

  if (local_loop)
  {
    robot_i = 0;
  }
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      forward_viewpoint_count++;
    }
  }
  if (local_loop)
  {
    robot_i = local_path.nodes_.size() - 1;
  }
  for (int i = robot_i - 1; i >= 0; i--)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      backward_viewpoint_count++;
    }
  }

  Eigen::Vector3d forward_lookahead_point = robot_position;
  Eigen::Vector3d backward_lookahead_point = robot_position;

  bool has_forward = false;
  bool has_backward = false;

  if (local_loop)
  {
    robot_i = 0;
  }
  bool forward_lookahead_point_in_los = true;
  bool backward_lookahead_point_in_los = true;
  double length_from_robot = 0.0;
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
  {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_).norm();
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i < local_path.GetNodeNum() - 1)
    {
      in_line_of_sight = pd_.viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i + 1].position_);
    }
    if ((length_from_robot > pp_.kLookAheadDistance || (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == local_path.GetNodeNum() - 1))

    {
      if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        forward_lookahead_point_in_los = false;
      }
      forward_lookahead_point = local_path.nodes_[i].position_;
      has_forward = true;
      break;
    }
  }
  if (local_loop)
  {
    robot_i = local_path.nodes_.size() - 1;
  }
  length_from_robot = 0.0;
  for (int i = robot_i - 1; i >= 0; i--)
  {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_).norm();
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i > 0)
    {
      in_line_of_sight = pd_.viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i - 1].position_);
    }
    if ((length_from_robot > pp_.kLookAheadDistance || (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END || i == 0))

    {
      if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        backward_lookahead_point_in_los = false;
      }
      backward_lookahead_point = local_path.nodes_[i].position_;
      has_backward = true;
      break;
    }
  }

  if (forward_viewpoint_count > 0 && !has_forward)
  {
    std::cout << "forward viewpoint count > 0 but does not have forward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }
  if (backward_viewpoint_count > 0 && !has_backward)
  {
    std::cout << "backward viewpoint count > 0 but does not have backward "
                 "lookahead point"
              << std::endl;
    exit(1);
  }

  double dx = pd_.lookahead_point_direction_.x();
  double dy = pd_.lookahead_point_direction_.y();

  if(reset_waypoint_){
    reset_waypoint_ = false;
    double lx = 1.0;
    double ly = 0.0;

    dx = cos(pd_.robot_yaw_) * lx - sin(pd_.robot_yaw_) * ly;
    dy = sin(pd_.robot_yaw_) * lx + cos(pd_.robot_yaw_) * ly;
  }
  

  double forward_angle_score = -2;
  double backward_angle_score = -2;
  double lookahead_angle_score = -2;

  bool forward_matches_completed_branch = false;
  bool backward_matches_completed_branch = false;
  int active_branch_anchor_stack_index = -1;
  if (!recovery_mode_)
  {
    active_branch_anchor_stack_index =
        GetActiveBranchAnchorStackIndex(robot_position, pp_.kCompletedBranchAnchorActiveDist);
  }
  if (active_branch_anchor_stack_index >= 0 && active_branch_anchor_stack_index < branch_anchors_.size())
  {
    const BranchAnchorState& active_anchor = branch_anchors_[active_branch_anchor_stack_index];
    if (has_forward)
    {
      forward_matches_completed_branch = DirectionMatchesCompletedBranch(active_anchor, forward_lookahead_point);
    }
    if (has_backward)
    {
      backward_matches_completed_branch = DirectionMatchesCompletedBranch(active_anchor, backward_lookahead_point);
    }
  }

  double dist_robot_to_lookahead = 0.0;
  bool prev_lookahead_point_in_los = true;
  if (has_forward)
  {
    Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
    forward_diff.z() = 0.0;
    forward_diff = forward_diff.normalized();
    forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y();
  }
  if (has_backward)
  {
    Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
    backward_diff.z() = 0.0;
    backward_diff = backward_diff.normalized();
    backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y();
  }
  if (has_lookahead)
  {
    Eigen::Vector3d prev_lookahead_point = local_path.nodes_[lookahead_i].position_;
    dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
    Eigen::Vector3d diff = prev_lookahead_point - robot_position;
    diff.z() = 0.0;
    diff = diff.normalized();
    lookahead_angle_score = dx * diff.x() + dy * diff.y();
    prev_lookahead_point_in_los = pd_.viewpoint_manager_->InCurrentFrameLineOfSight(prev_lookahead_point);
  }

  pd_.lookahead_point_cloud_->cloud_->clear();

  if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0)
  {
    relocation_ = true;
  }
  else
  {
    relocation_ = false;
  }

  bool keep_previous_lookahead = false;
  if (!relocation_ && has_lookahead)
  {
    double best_candidate_score = forward_angle_score > backward_angle_score ? forward_angle_score : backward_angle_score;
    keep_previous_lookahead =
        dist_robot_to_lookahead > pp_.kLookAheadKeepMinDistance &&
        pd_.viewpoint_manager_->InLocalPlanningHorizon(local_path.nodes_[lookahead_i].position_) &&
        lookahead_angle_score + pp_.kLookAheadSwitchScoreMargin >= best_candidate_score;
  }

  if (active_branch_anchor_stack_index >= 0 && has_lookahead &&
      active_branch_anchor_stack_index < branch_anchors_.size() &&
      DirectionMatchesCompletedBranch(branch_anchors_[active_branch_anchor_stack_index],
                                      local_path.nodes_[lookahead_i].position_))
  {
    keep_previous_lookahead = false;
  }

  bool suppress_forward =
      forward_matches_completed_branch && !backward_matches_completed_branch && backward_viewpoint_count > 0;
  bool suppress_backward =
      backward_matches_completed_branch && !forward_matches_completed_branch && forward_viewpoint_count > 0;
  if (forward_matches_completed_branch && backward_matches_completed_branch)
  {
    suppress_forward = false;
    suppress_backward = false;
  }

  if (suppress_forward)
  {
    forward_viewpoint_count = 0;
    forward_angle_score = -2;
  }
  if (suppress_backward)
  {
    backward_viewpoint_count = 0;
    backward_angle_score = -2;
  }

  if (relocation_)
  {
    if (use_momentum_ && pp_.kUseMomentum)
    {
      if (forward_angle_score > backward_angle_score)
      {
        lookahead_point = forward_lookahead_point;
      }
      else
      {
        lookahead_point = backward_lookahead_point;
      }
    }
    else
    {
      // follow the shorter distance one
      if (dist_from_start < dist_from_end && local_path.nodes_.front().type_ != exploration_path_ns::NodeType::ROBOT)
      {
        lookahead_point = backward_lookahead_point;
      }
      else if (dist_from_end < dist_from_start &&
               local_path.nodes_.back().type_ != exploration_path_ns::NodeType::ROBOT)
      {
        lookahead_point = forward_lookahead_point;
      }
      else
      {
        lookahead_point =
            forward_angle_score > backward_angle_score ? forward_lookahead_point : backward_lookahead_point;
      }
    }
  }
  else if (keep_previous_lookahead)
  {
    lookahead_point = local_path.nodes_[lookahead_i].position_;
  }
  else
  {
    if (forward_angle_score > backward_angle_score)
    {
      if (forward_viewpoint_count > 0)
      {
        lookahead_point = forward_lookahead_point;
      }
      else
      {
        lookahead_point = backward_lookahead_point;
      }
    }
    else
    {
      if (backward_viewpoint_count > 0)
      {
        lookahead_point = backward_lookahead_point;
      }
      else
      {
        lookahead_point = forward_lookahead_point;
      }
    }
  }

  if ((lookahead_point == forward_lookahead_point && !forward_lookahead_point_in_los) ||
      (lookahead_point == backward_lookahead_point && !backward_lookahead_point_in_los))
  {
    lookahead_point_in_line_of_sight_ = false;
  }
  else if (has_lookahead && lookahead_point == local_path.nodes_[lookahead_i].position_)
  {
    lookahead_point_in_line_of_sight_ = prev_lookahead_point_in_los;
  }
  else
  {
    lookahead_point_in_line_of_sight_ = true;
  }

  pd_.lookahead_point_direction_ = lookahead_point - robot_position;
  pd_.lookahead_point_direction_.z() = 0.0;
  pd_.lookahead_point_direction_.normalize();

  pcl::PointXYZI point;
  point.x = lookahead_point.x();
  point.y = lookahead_point.y();
  point.z = lookahead_point.z();
  point.intensity = 1.0;
  pd_.lookahead_point_cloud_->cloud_->points.push_back(point);

  if (has_lookahead)
  {
    point.x = local_path.nodes_[lookahead_i].position_.x();
    point.y = local_path.nodes_[lookahead_i].position_.y();
    point.z = local_path.nodes_[lookahead_i].position_.z();
    point.intensity = 0;
    pd_.lookahead_point_cloud_->cloud_->points.push_back(point);
  }
  return true;
}

void SensorCoveragePlanner3D::PublishWaypoint()
{
  geometry_msgs::PointStamped waypoint;
  if (exploration_finished_ && near_home_ && pp_.kRushHome)
  {
    waypoint.point.x = pd_.initial_position_.x();
    waypoint.point.y = pd_.initial_position_.y();
    waypoint.point.z = pd_.initial_position_.z();
  }
  else
  {
    double dx = pd_.lookahead_point_.x() - pd_.robot_position_.x;
    double dy = pd_.lookahead_point_.y() - pd_.robot_position_.y;
    double r = sqrt(dx * dx + dy * dy);
    double extend_dist =
        lookahead_point_in_line_of_sight_ ? pp_.kExtendWayPointDistanceBig : pp_.kExtendWayPointDistanceSmall;
    if (r < extend_dist && pp_.kExtendWayPoint)
    {
      dx = dx / r * extend_dist;
      dy = dy / r * extend_dist;
    }
    waypoint.point.x = dx + pd_.robot_position_.x;
    waypoint.point.y = dy + pd_.robot_position_.y;
    waypoint.point.z = pd_.lookahead_point_.z();
  }
  misc_utils_ns::Publish<geometry_msgs::PointStamped>(waypoint_pub_, waypoint, kWorldFrameID);
}

void SensorCoveragePlanner3D::PublishRecoveryWaypoint()
{
  geometry_msgs::PointStamped waypoint;
  waypoint.point.x = recovery_waypoint_.x();
  waypoint.point.y = recovery_waypoint_.y();
  waypoint.point.z = recovery_waypoint_.z();
  misc_utils_ns::Publish<geometry_msgs::PointStamped>(waypoint_pub_, waypoint, kWorldFrameID);
}

void SensorCoveragePlanner3D::PublishRuntime()
{
  local_viewpoint_sampling_runtime_ = pd_.local_coverage_planner_->GetViewPointSamplingRuntime() / 1000;
  local_path_finding_runtime_ =
      (pd_.local_coverage_planner_->GetFindPathRuntime() + pd_.local_coverage_planner_->GetTSPRuntime()) / 1000;

  std_msgs::Int32MultiArray runtime_breakdown_msg;
  runtime_breakdown_msg.data.clear();
  runtime_breakdown_msg.data.push_back(update_representation_runtime_);
  runtime_breakdown_msg.data.push_back(local_viewpoint_sampling_runtime_);
  runtime_breakdown_msg.data.push_back(local_path_finding_runtime_);
  runtime_breakdown_msg.data.push_back(global_planning_runtime_);
  runtime_breakdown_msg.data.push_back(trajectory_optimization_runtime_);
  runtime_breakdown_msg.data.push_back(overall_runtime_);
  runtime_breakdown_pub_.publish(runtime_breakdown_msg);

  float runtime = 0;
  if (!exploration_finished_ && pp_.kNoExplorationReturnHome)
  {
    for (int i = 0; i < runtime_breakdown_msg.data.size() - 1; i++)
    {
      runtime += runtime_breakdown_msg.data[i];
    }
  }

  std_msgs::Float32 runtime_msg;
  runtime_msg.data = runtime / 1000.0;
  runtime_pub_.publish(runtime_msg);
}

double SensorCoveragePlanner3D::GetRobotToHomeDistance()
{
  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  return (robot_position - pd_.initial_position_).norm();
}

void SensorCoveragePlanner3D::PublishExplorationState()
{
  std_msgs::Bool exploration_finished_msg;
  exploration_finished_msg.data = exploration_finished_;
  exploration_finish_pub_.publish(exploration_finished_msg);
}

void SensorCoveragePlanner3D::PublishStuckRecoveryMode()
{
  std_msgs::Bool recovery_mode_msg;
  recovery_mode_msg.data = stuck_recovery_mode_;
  stuck_recovery_mode_pub_.publish(recovery_mode_msg);
}

void SensorCoveragePlanner3D::PrintExplorationStatus(std::string status, bool clear_last_line)
{
  if (clear_last_line)
  {
    printf(cursup);
    printf(cursclean);
    printf(cursup);
    printf(cursclean);
  }
  std::cout << std::endl << "\033[1;32m" << status << "\033[0m" << std::endl;
}

void SensorCoveragePlanner3D::CountDirectionChange()
{
  Eigen::Vector3d current_moving_direction_ =
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z) -
      Eigen::Vector3d(pd_.last_robot_position_.x, pd_.last_robot_position_.y, pd_.last_robot_position_.z);

  if (current_moving_direction_.norm() > 0.5)
  {
    if (pd_.moving_direction_.dot(current_moving_direction_) < 0)
    {
      direction_change_count_++;
      direction_no_change_count_ = 0;
      if (direction_change_count_ > pp_.kDirectionChangeCounterThr)
      {
        if (!use_momentum_)
        {
          momentum_activation_count_++;
        }
        use_momentum_ = true;
      }
    }
    else
    {
      direction_no_change_count_++;
      if (direction_no_change_count_ > pp_.kDirectionNoChangeCounterThr)
      {
        direction_change_count_ = 0;
        use_momentum_ = false;
      }
    }
    pd_.moving_direction_ = current_moving_direction_;
  }
  pd_.last_robot_position_ = pd_.robot_position_;

  std_msgs::Int32 momentum_activation_count_msg;
  momentum_activation_count_msg.data = momentum_activation_count_;
  momentum_activation_count_pub_.publish(momentum_activation_count_msg);
}

void SensorCoveragePlanner3D::execute(const ros::TimerEvent&)
{
  if (!pp_.kAutoStart && !start_exploration_)
  {
    ROS_INFO("Waiting for start signal");
    return;
  }
  Timer overall_processing_timer("overall processing");
  update_representation_runtime_ = 0;
  local_viewpoint_sampling_runtime_ = 0;
  local_path_finding_runtime_ = 0;
  global_planning_runtime_ = 0;
  trajectory_optimization_runtime_ = 0;
  overall_runtime_ = 0;

  if (!initialized_)
  {
    SendInitialWaypoint();
    start_time_ = ros::Time::now();
    global_direction_switch_time_ = ros::Time::now();
    return;
  }

  overall_processing_timer.Start();
  if (keypose_cloud_update_)
  {
    keypose_cloud_update_ = false;

    CountDirectionChange();

    misc_utils_ns::Timer update_representation_timer("update representation");
    update_representation_timer.Start();

    // Update grid world
    UpdateGlobalRepresentation();

    int viewpoint_candidate_count = UpdateViewPoints();
    if (viewpoint_candidate_count == 0)
    {
      ROS_WARN("Cannot get candidate viewpoints, skipping this round");
      return;
    }
    UpdateBranchAnchors();

    UpdateKeyposeGraph();

    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    if (!exploration_finished_ || !pp_.kNoExplorationReturnHome)
    {
      UpdateViewPointCoverage();
      UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);
    }
    else
    {
      pd_.viewpoint_manager_->ResetViewPointCoverage();
    }

    update_representation_timer.Stop(false);
    update_representation_runtime_ += update_representation_timer.GetDuration("ms");

    // Global TSP
    std::vector<int> global_cell_tsp_order;
    exploration_path_ns::ExplorationPath global_path;
    GlobalPlanning(global_cell_tsp_order, global_path);

    // Local TSP
    exploration_path_ns::ExplorationPath local_path;
    LocalPlanning(uncovered_point_num, uncovered_frontier_point_num, global_path, local_path);

    near_home_ = GetRobotToHomeDistance() < pp_.kRushHomeDist;
    at_home_ = GetRobotToHomeDistance() < pp_.kAtHomeDistThreshold;

    bool return_home_candidate =
        pd_.grid_world_->IsReturningHome() && pd_.local_coverage_planner_->IsLocalCoverageComplete() &&
        (ros::Time::now() - start_time_).toSec() > 5;
    if (!recovery_mode_ && pd_.local_coverage_planner_->IsLocalCoverageComplete() && !pd_.grid_world_->IsReturningHome())
    {
      StartRecoveryToLatestAnchor("completed current dead-end branch");
    }
    if (recovery_mode_)
    {
      return_home_candidate = false;
    }
    if (return_home_candidate)
    {
      return_home_candidate_count_++;
    }
    else
    {
      return_home_candidate_count_ = 0;
    }

    bool return_home_confirmed =
        return_home_candidate && return_home_candidate_count_ >= pp_.kReturnHomeCandidateCountThreshold;

    if (return_home_candidate && !return_home_confirmed)
    {
      global_path = exploration_path_ns::ExplorationPath();
      local_path = exploration_path_ns::ExplorationPath();
    }

    if (return_home_confirmed)
    {
      if (!exploration_finished_)
      {
        PrintExplorationStatus("Exploration completed, returning home", false);
      }
      exploration_finished_ = true;
    }

    if (exploration_finished_ && at_home_ && !stopped_)
    {
      PrintExplorationStatus("Return home completed", false);
      stopped_ = true;
    }

    if (!return_home_candidate || return_home_confirmed || pd_.exploration_path_.GetNodeNum() == 0)
    {
      pd_.exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);
      lookahead_point_update_ = GetLookAheadPoint(pd_.exploration_path_, global_path, pd_.lookahead_point_);
    }

    if (!recovery_mode_)
    {
      UpdateProgressState(pd_.lookahead_point_, lookahead_point_update_);
      if (stuck_cycle_count_ >= pp_.kStuckCycleThreshold)
      {
        StartRecoveryToLatestAnchor("robot progress stalled");
      }
    }

    bool has_recovery_waypoint = false;
    if (recovery_mode_)
    {
      has_recovery_waypoint = UpdateRecoveryWaypoint();
      if (!has_recovery_waypoint)
      {
        ExitRecoveryMode(false);
      }
    }

    PublishExplorationState();
    PublishStuckRecoveryMode();
    if (has_recovery_waypoint)
    {
      PublishRecoveryWaypoint();
    }
    else
    {
      PublishWaypoint();
    }

    overall_processing_timer.Stop(false);
    overall_runtime_ = overall_processing_timer.GetDuration("ms");

    pd_.visualizer_->GetGlobalSubspaceMarker(pd_.grid_world_, global_cell_tsp_order);
    Eigen::Vector3d viewpoint_origin = pd_.viewpoint_manager_->GetOrigin();
    pd_.visualizer_->GetLocalPlanningHorizonMarker(viewpoint_origin.x(), viewpoint_origin.y(), pd_.robot_position_.z);
    pd_.visualizer_->PublishMarkers();

    PublishLocalPlanningVisualization(local_path);
    PublishGlobalPlanningVisualization(global_path, local_path);
    PublishRuntime();
  }
}
}  // namespace sensor_coverage_planner_3d_ns
