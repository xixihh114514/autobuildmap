#ifndef RRT_GOAL_DECISION_FRONTIER_PARAMS_H
#define RRT_GOAL_DECISION_FRONTIER_PARAMS_H

#include <ros/ros.h>

namespace frontier_params {

struct FrontierParams {
    // 基础参数
    double cluster_radius = 0.3;
    double goal_check_interval = 0.5;
    double distance_change_threshold = 0.1;
    double abort_timeout = 60.0;
    double plan_timeout = 5.0;
    int slow_count_limit = 8;
    double cancel_timeout = 3.0;

    // 权重参数
    double num_weight = 2.0;
    double distance_weight = -4.0;

    // 黑名单参数
    double failed_frontier_tolerance = 0.3;
    double failed_frontier_timeout = 40.0;
    double failed_frontier_cool_down = 5.0;

    // 搜索参数
    double orientation_search_radius = 0.25;
    double known_area_search_radius = 0.5;
    double expected_unknow_proportion = 0.7;
    double known_area_punishment = 1.0;
    double obstacle_check_interval = 5.0;

    // 障碍物检测参数
    double nearby_obstacle_threshold = 80.0;
    double nearby_obstacle_confirm_delay = 3.0;
    double nearby_check_radius = 3.0;

    // Shrink 参数
    double global_extra_shrink_distance = 0.2;
    double global_shrink_radius = 1.5;
    double global_shrink_step = 0.05;
    double global_safe_threshold = 50.0;
    int global_safe_window_pixels = 1;

    // 到达判断参数
    double arrived_ignore_radius = 0.8;

    // 救援模式参数
    double rescue_trigger_timeout = 10.0;
    double rescue_search_radius = 2.0;
    double rescue_max_global_cost = 100.0;
    double rescue_plan_timeout = 3.0;
    int rescue_max_attempts = 20;
    int rescue_trigger_failures = 5;
    int rescue_trigger_count_threshold = 10;
    double rescue_goal_timeout = 15.0;
};

// 从参数服务器加载参数
FrontierParams loadParams(ros::NodeHandle& pnh);

// 打印参数信息
void printParams(const FrontierParams& params);

} // namespace frontier_params

#endif // RRT_GOAL_DECISION_FRONTIER_PARAMS_H
