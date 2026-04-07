#ifndef RRT_GOAL_DECISION_FRONTIER_GOAL_MANAGER_H
#define RRT_GOAL_DECISION_FRONTIER_GOAL_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <angles/angles.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <random>
#include <mutex>
#include <iomanip>

#include <std_msgs/Float32MultiArray.h>
#include <map_msgs/OccupancyGridUpdate.h>

#include <rrt_goal_decision/frontier_utils.h>
#include <rrt_goal_decision/frontier_params.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct FrontierCluster
{
    double x = 0.0;
    double y = 0.0;
    int num = 0;
    double cost = 254.0;
    double distance = 0.0;
    double weight = -1e9;
};

struct FailedFrontier
{
    double x = 0.0;
    double y = 0.0;
    ros::Time stamp;
    ros::Time cool_until;

    FailedFrontier() : x(0.0), y(0.0), stamp(ros::Time::now()), cool_until(ros::Time::now()) {}
    FailedFrontier(double _x, double _y, const ros::Time& _stamp, const ros::Time& _cool_until)
        : x(_x), y(_y), stamp(_stamp), cool_until(_cool_until) {}
};

class FrontierGoalManager
{
public:
    FrontierGoalManager();
    void spin();

private:
    // ROS 相关
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber frontier_sub;
    ros::Subscriber global_costmap_sub;
    ros::Subscriber costmap_updates_sub;
    ros::Subscriber plan_sub;
    ros::Subscriber map_sub_;

    tf::TransformListener tf_listener;
    MoveBaseClient ac;

    ros::Publisher candidate_pub;
    ros::Publisher rescue_info_pub;

    // 地图相关
    nav_msgs::OccupancyGrid global_costmap;
    nav_msgs::OccupancyGridConstPtr current_map_;
    std::mutex map_mutex_;
    bool has_raw_map_;
    bool has_global_costmap;

    // 状态管理
    std::vector<geometry_msgs::PointStamped> frontier_buffer;
    std::vector<FailedFrontier> failed_frontiers;

    bool goal_active;
    bool cancel_pending;
    bool just_canceled;
    bool is_rescue_mode;
    move_base_msgs::MoveBaseGoal current_goal;

    ros::Time goal_start_time;
    ros::Time goal_send_time;
    ros::Time cancel_request_time;

    double rescue_center_x;
    double rescue_center_y;
    int rescue_attempt_count;

    ros::Time rescue_goal_start_time;
    double rescue_goal_timeout;

    ros::Time rescue_mode_start_time;
    double rescue_mode_total_duration;

    ros::Time last_stuck_abort_check_time;
    geometry_msgs::Point last_robot_position;

    int consecutive_plan_failures;
    int rescue_trigger_count;  // 统计三种触发救援的情况：move_base ABORTED、Robot Stuck、Goal Total Timeout

    double last_distance;
    ros::Time last_distance_time;
    bool distance_initialized;

    ros::Time last_obstacle_check_time;
    bool obstacle_suspected;
    ros::Time first_obstacle_detect_time;

    ros::Time last_revive_warn_time;

    bool plan_received;
    bool waiting_for_plan;
    int slow_count;

    // 参数（统一存储在结构体中）
    frontier_params::FrontierParams params;

    std::mt19937 gen;

    // 回调函数
    void frontierCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void costmapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
    void rawMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void planCallback(const nav_msgs::Path::ConstPtr& msg);

    // 辅助函数
    void publishRescueInfo();
    void resetRescueModeState();
    void computeRescueCenter();
    void activateRescueMode();

    void markGoalAsFailed(double x, double y, double cool_down_multiplier = 1.0);
    void incrementRescueTriggerCount(const std::string& reason);

    // 核心功能函数
    bool getRobotPose(double &x, double &y, double &yaw);
    bool shrinkOnGlobalCostmap(double &target_x, double &target_y, double robot_x, double robot_y);
    bool isGlobalCostValid(double x, double y);
    bool checkNearbyObstacleRunning();
    bool isNearbyCostSafe(double x, double y);
    double getCost(double x, double y);
    double calculateUnknownProportion(double x, double y);
    std::vector<FrontierCluster> clusterFrontiers();
    double calculateFrontierOrientation(double x, double y, double rx, double ry);
    FrontierCluster selectGoal(bool force_revive);
    bool attemptRescueGoal();
    void sendGoal(const FrontierCluster &c);
    void requestCancel();
    void finalizeCancel();
    void monitorGoal();
};

#endif // RRT_GOAL_DECISION_FRONTIER_GOAL_MANAGER_H
