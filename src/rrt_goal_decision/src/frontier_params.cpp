#include "rrt_goal_decision/frontier_params.h"

namespace frontier_params {

FrontierParams loadParams(ros::NodeHandle& pnh)
{
    FrontierParams params;

    pnh.param("cluster_radius", params.cluster_radius, 0.3);
    pnh.param("goal_check_interval", params.goal_check_interval, 0.5);
    pnh.param("distance_change_threshold", params.distance_change_threshold, 0.1);
    pnh.param("abort_timeout", params.abort_timeout, 60.0);
    pnh.param("plan_timeout", params.plan_timeout, 5.0);
    pnh.param("slow_count_limit", params.slow_count_limit, 8);
    pnh.param("cancel_timeout", params.cancel_timeout, 3.0);

    pnh.param("num_weight", params.num_weight, 2.0);
    pnh.param("distance_weight", params.distance_weight, -4.0);

    pnh.param("failed_frontier_tolerance", params.failed_frontier_tolerance, 0.3);
    pnh.param("failed_frontier_timeout", params.failed_frontier_timeout, 40.0);
    pnh.param("failed_frontier_cool_down", params.failed_frontier_cool_down, 5.0);

    pnh.param("orientation_search_radius", params.orientation_search_radius, 0.25);
    pnh.param("known_area_search_radius", params.known_area_search_radius, 0.5);
    pnh.param("expected_unknow_proportion", params.expected_unknow_proportion, 0.7);
    pnh.param("known_area_punishment", params.known_area_punishment, 1.0);
    pnh.param("obstacle_check_interval", params.obstacle_check_interval, 5.0);

    pnh.param("nearby_obstacle_threshold", params.nearby_obstacle_threshold, 80.0);
    pnh.param("nearby_obstacle_confirm_delay", params.nearby_obstacle_confirm_delay, 3.0);
    pnh.param("nearby_check_radius", params.nearby_check_radius, 3.0);

    pnh.param("global_extra_shrink_distance", params.global_extra_shrink_distance, 0.2);
    pnh.param("global_shrink_radius", params.global_shrink_radius, 1.5);
    pnh.param("global_shrink_step", params.global_shrink_step, 0.05);
    pnh.param("global_safe_threshold", params.global_safe_threshold, 50.0);
    pnh.param("global_safe_window_pixels", params.global_safe_window_pixels, 1);

    pnh.param("arrived_ignore_radius", params.arrived_ignore_radius, 0.8);

    pnh.param("rescue_trigger_timeout", params.rescue_trigger_timeout, 10.0);
    pnh.param("rescue_search_radius", params.rescue_search_radius, 2.0);
    pnh.param("rescue_max_global_cost", params.rescue_max_global_cost, 100.0);
    pnh.param("rescue_plan_timeout", params.rescue_plan_timeout, 3.0);
    pnh.param("rescue_max_attempts", params.rescue_max_attempts, 20);
    pnh.param("rescue_trigger_failures", params.rescue_trigger_failures, 5);
    pnh.param("rescue_trigger_count_threshold", params.rescue_trigger_count_threshold, 10);
    pnh.param("rescue_goal_timeout", params.rescue_goal_timeout, 15.0);

    return params;
}

void printParams(const FrontierParams& params)
{
    ROS_INFO("========================================");
    ROS_INFO("[V5.7] Rescue Mode Configuration:");
    ROS_INFO("  - rescue_trigger_failures: %d (consecutive plan failures)", params.rescue_trigger_failures);
    ROS_INFO("  - rescue_trigger_count_threshold: %d (stuck/abort counter)", params.rescue_trigger_count_threshold);
    ROS_INFO("  - rescue_goal_timeout: %.1fs (per goal timeout)", params.rescue_goal_timeout);
    ROS_INFO("  - rescue_max_attempts: %d", params.rescue_max_attempts);
    ROS_INFO("  - global_safe_threshold: %.1f", params.global_safe_threshold);
    ROS_INFO("  - global_shrink_radius: %.2fm", params.global_shrink_radius);
    ROS_INFO("  - global_extra_shrink_distance: %.2fm", params.global_extra_shrink_distance);
    ROS_INFO("========================================");
}

} // namespace frontier_params
