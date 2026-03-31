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
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber frontier_sub;
    ros::Subscriber global_costmap_sub;
    ros::Subscriber costmap_updates_sub;  // 【新增】全局代价地图增量更新订阅
    ros::Subscriber plan_sub;
    ros::Subscriber map_sub_;

    tf::TransformListener tf_listener;
    MoveBaseClient ac;

    nav_msgs::OccupancyGrid global_costmap;
    
    nav_msgs::OccupancyGridConstPtr current_map_; 
    std::mutex map_mutex_; 
    bool has_raw_map_;

    bool has_global_costmap;

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

    ros::Time rescue_start_time;
    double rescue_center_x;
    double rescue_center_y;
    int rescue_attempt_count;

    int consecutive_plan_failures;  // 【新增】连续 plan failed 计数

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

    /* 参数 */
    double cluster_radius;
    double goal_check_interval;
    double distance_change_threshold;
    double abort_timeout;
    double plan_timeout;
    int slow_count_limit;
    double cancel_timeout;

    double num_weight;
    double distance_weight;
    
    double failed_frontier_tolerance;
    double failed_frontier_timeout;
    double failed_frontier_cool_down;
    double orientation_search_radius; 
    double known_area_search_radius;
    double expected_unknow_proportion;
    double known_area_punishment;
    double obstacle_check_interval;      
    
    double nearby_obstacle_threshold;
    double nearby_obstacle_confirm_delay; 
    double nearby_check_radius; 
    
    // 【新增】shrink 完成后额外收缩距离
    double global_extra_shrink_distance;

    double global_shrink_radius;      
    double global_shrink_step;        
    double global_safe_threshold;     
    double global_safe_window_resolution;  // 【新增】用于计算像素检测半径

    double rescue_trigger_timeout;
    double rescue_search_radius;
    double rescue_max_global_cost;
    double rescue_plan_timeout;
    int rescue_max_attempts;
    int rescue_trigger_failures;  // 【新增】触发救援的连续 failed 次数阈值

    double arrived_ignore_radius;

    std::mt19937 gen;

    ros::Publisher candidate_pub;
    ros::Publisher rescue_info_pub;

public:
    FrontierGoalManager() : pnh("~"), ac("move_base", true), gen(std::random_device{}())
    {
        pnh.param("cluster_radius", cluster_radius, 0.3);
        pnh.param("goal_check_interval", goal_check_interval, 0.5);
        pnh.param("distance_change_threshold", distance_change_threshold, 0.1);
        pnh.param("abort_timeout", abort_timeout, 60.0);
        pnh.param("plan_timeout", plan_timeout, 5.0);
        pnh.param("slow_count_limit", slow_count_limit, 8);
        pnh.param("cancel_timeout", cancel_timeout, 3.0);

        pnh.param("num_weight", num_weight, 2.0);
        pnh.param("distance_weight", distance_weight, -4.0);
        
        pnh.param("failed_frontier_tolerance", failed_frontier_tolerance, 0.3);
        pnh.param("failed_frontier_timeout", failed_frontier_timeout, 40.0);
        pnh.param("failed_frontier_cool_down", failed_frontier_cool_down, 5.0);
        pnh.param("orientation_search_radius", orientation_search_radius, 0.25);
        pnh.param("known_area_search_radius", known_area_search_radius, 0.5);
        pnh.param("expected_unknow_proportion", expected_unknow_proportion, 0.7);
        pnh.param("known_area_punishment", known_area_punishment, 1.0);
        pnh.param("obstacle_check_interval", obstacle_check_interval, 5.0);
        
        pnh.param("nearby_obstacle_threshold", nearby_obstacle_threshold, 80.0); 
        pnh.param("nearby_obstacle_confirm_delay", nearby_obstacle_confirm_delay, 3.0); 
        pnh.param("nearby_check_radius", nearby_check_radius, 3.0); 
        
        pnh.param("global_extra_shrink_distance", global_extra_shrink_distance, 0.2);  // 【新增】
        
        pnh.param("global_shrink_radius", global_shrink_radius, 1.5);   
        pnh.param("global_shrink_step", global_shrink_step, 0.05);      
        pnh.param("global_safe_threshold", global_safe_threshold, 50.0);  // 恢复默认阈值 
        pnh.param("global_safe_window_resolution", global_safe_window_resolution, 0.03);  // 【新增】

        pnh.param("arrived_ignore_radius", arrived_ignore_radius, 0.8); 

        pnh.param("rescue_trigger_timeout", rescue_trigger_timeout, 10.0);
        pnh.param("rescue_search_radius", rescue_search_radius, 2.0);
        pnh.param("rescue_max_global_cost", rescue_max_global_cost, 100.0);
        pnh.param("rescue_plan_timeout", rescue_plan_timeout, 3.0);
        pnh.param("rescue_max_attempts", rescue_max_attempts, 20);
        pnh.param("rescue_trigger_failures", rescue_trigger_failures, 5);  // 【新增】默认 5 次连续 failed 触发救援

        ROS_INFO("========================================");
        ROS_INFO("[V5.3] Rescue Mode Configuration:");
        ROS_INFO("  - rescue_trigger_failures: %d", rescue_trigger_failures);
        ROS_INFO("  - rescue_max_attempts: %d", rescue_max_attempts);
        ROS_INFO("  - global_safe_threshold: %.1f", global_safe_threshold);
        ROS_INFO("  - global_shrink_radius: %.2fm", global_shrink_radius);
        ROS_INFO("  - global_extra_shrink_distance: %.2fm", global_extra_shrink_distance);
        ROS_INFO("========================================");

        has_global_costmap = false;
        has_raw_map_ = false;

        frontier_sub = nh.subscribe("/detected_points", 100, &FrontierGoalManager::frontierCallback, this);
        global_costmap_sub = nh.subscribe("/move_base/global_costmap/costmap", 10, &FrontierGoalManager::globalCostmapCallback, this);
        costmap_updates_sub = nh.subscribe("/move_base/global_costmap/costmap_updates", 10, &FrontierGoalManager::costmapUpdatesCallback, this);  // 【新增】订阅增量更新
        plan_sub = nh.subscribe("/move_base/GlobalPlanner/plan", 10, &FrontierGoalManager::planCallback, this);
        map_sub_ = nh.subscribe("/map", 10, &FrontierGoalManager::rawMapCallback, this);
        
        candidate_pub = nh.advertise<std_msgs::Float32MultiArray>("/goal_candidates", 1);
        rescue_info_pub = nh.advertise<std_msgs::Float32MultiArray>("/rescue_mode_info", 1, true);

        goal_active = false;
        cancel_pending = false;
        just_canceled = false;
        is_rescue_mode = false;
        plan_received = false;
        waiting_for_plan = false;
        slow_count = 0;
        distance_initialized = false;
        rescue_attempt_count = 0;
        consecutive_plan_failures = 0;  // 【新增】初始化连续 failed 计数

        ROS_INFO("Waiting for move_base...");
        if (!ac.waitForServer(ros::Duration(10.0))) {
            ROS_ERROR("move_base server not found.");
            ros::shutdown();
            return;
        }
        
        publishRescueInfo();  // 【新增】初始发布救援信息
        ROS_INFO("Frontier Explorer Ready (V5.3 - Failure-Based Rescue).");
    }

    // 【新增】发布救援模式信息
    void publishRescueInfo()
    {
        std_msgs::Float32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = is_rescue_mode ? 1.0 : 0.0;  // 是否在救援模式
        msg.data[1] = rescue_center_x;              // 救援中心 X
        msg.data[2] = rescue_center_y;              // 救援中心 Y
        msg.data[3] = static_cast<float>(rescue_attempt_count);  // 救援尝试次数
        rescue_info_pub.publish(msg);
    }

    void frontierCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        if (frontier_buffer.size() > 2000) frontier_buffer.erase(frontier_buffer.begin());
        frontier_buffer.push_back(*msg);
    }

    void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        global_costmap = *msg;
        has_global_costmap = true;
        ROS_INFO("[COSTMAP] Received initial global costmap: %dx%d, resolution=%.3f",
                 msg->info.width, msg->info.height, msg->info.resolution);
    }
    
    // 【新增】处理全局代价地图增量更新
    void costmapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
    {
        if (!has_global_costmap)
        {
            ROS_WARN("[COSTMAP_UPDATE] Received update before initial map!");
            return;
        }
        
        // 检查更新范围是否有效
        if (msg->x < 0 || msg->y < 0 || 
            msg->x + msg->width > global_costmap.info.width ||
            msg->y + msg->height > global_costmap.info.height)
        {
            ROS_ERROR("[COSTMAP_UPDATE] Update out of bounds!");
            return;
        }
        
        // 更新地图数据
        int updates = 0;
        for (size_t i = 0; i < msg->data.size(); ++i)
        {
            int x = (msg->x + i % msg->width);
            int y = (msg->y + i / msg->width);
            int idx = y * global_costmap.info.width + x;
            
            if (idx >= 0 && idx < global_costmap.data.size())
            {
                global_costmap.data[idx] = msg->data[i];
                updates++;
            }
        }
        
        ROS_INFO_THROTTLE(10.0, "[COSTMAP_UPDATE] Applied %d updates at (%d,%d) size=%lux%lu",
                         updates, msg->x, msg->y, msg->width, msg->height);
    }

    void rawMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = msg;
        has_raw_map_ = true;
    }

    void planCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (!goal_active || plan_received || msg->poses.empty()) return;
        plan_received = true;
        waiting_for_plan = false;
        goal_start_time = ros::Time::now();
        slow_count = 0;
    }

    bool getRobotPose(double &x, double &y, double &yaw)
    {
        tf::StampedTransform transform;
        try {
            if (tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.05))) {
                tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
            } else { return false; }
        } catch(tf::TransformException &ex) { return false; }
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());
        return true;
    }

    // 【修复版】shrinkOnGlobalCostmap
    // 1. 使用 3x3 像素检测（根据地图分辨率动态计算）
    // 2. 增加额外收缩距离参数
    bool shrinkOnGlobalCostmap(double &target_x, double &target_y, double robot_x, double robot_y)
    {
        if (!has_global_costmap || global_costmap.data.empty()) {
            ROS_WARN("[SHRINK] No costmap available.");
            return false;
        }

        double res = global_costmap.info.resolution;
        if (res <= 1e-6) {
            ROS_WARN("[SHRINK] Invalid map resolution.");
            return false;
        }

        double map_origin_x = global_costmap.info.origin.position.x;
        double map_origin_y = global_costmap.info.origin.position.y;
        int width = global_costmap.info.width;
        int height = global_costmap.info.height;

        // 【修复】使用 3x3 像素检测，而不是固定半径
        int safe_window_pixels = 1;  // 3x3 窗口，中心点±1 像素
        double safe_window_radius = global_safe_window_resolution * 1.5;  // 用于日志

        int mx = static_cast<int>((target_x - map_origin_x) / res);
        int my = static_cast<int>((target_y - map_origin_y) / res);

        if (mx < 0 || mx >= width || my < 0 || my >= height) {
            ROS_WARN("[SHRINK] Target (%.2f, %.2f) out of map bounds. mx=%d, my=%d, map=%dx%d",
                     target_x, target_y, mx, my, width, height);
            return false;
        }

        // 【调试】检查初始点的代价值
        int center_index = my * width + mx;
        int8_t raw_center = global_costmap.data[center_index];
        unsigned char center_cost = (raw_center < 0) ? 255 : static_cast<unsigned char>(raw_center);
        ROS_INFO("[SHRINK] Checking target (%.2f, %.2f), map_idx=(%d,%d), center_cost=%d, threshold=%d",
                 target_x, target_y, mx, my, (int)center_cost, (int)global_safe_threshold);

        // 【修复】检查 3x3 像素区域
        bool is_safe = true;
        int unsafe_count = 0;
        for (int dy = -safe_window_pixels; dy <= safe_window_pixels && is_safe; ++dy) {
            for (int dx = -safe_window_pixels; dx <= safe_window_pixels && is_safe; ++dx) {
                int nx = mx + dx;
                int ny = my + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    is_safe = false;
                    break;
                }
                int index = ny * width + nx;
                if (index >= global_costmap.data.size()) {
                    is_safe = false;
                    break;
                }
                unsigned char cost = static_cast<unsigned char>(global_costmap.data[index]);
                if (cost >= global_safe_threshold) {
                    is_safe = false;
                    unsafe_count++;
                }
            }
        }

        if (is_safe) {
            ROS_INFO("[SHRINK] Target is already safe. cost=%d", (int)center_cost);
            return true;  // 已经是安全的
        }

        ROS_INFO("[SHRINK] Target unsafe (unsafe_pixels=%d). Starting shrink...", unsafe_count);

        // 需要 shrink
        double dx = robot_x - target_x;
        double dy = robot_y - target_y;
        double dist = hypot(dx, dy);
        if (dist < 1e-6) {
            ROS_WARN("[SHRINK] Robot and target too close (dist=%.3f).", dist);
            return false;
        }

        double dir_x = dx / dist;
        double dir_y = dy / dist;

        double current_shrink_dist = 0.0;
        double new_x = target_x;
        double new_y = target_y;
        bool found_safe = false;

        while (current_shrink_dist < global_shrink_radius) {
            new_x += dir_x * global_shrink_step;
            new_y += dir_y * global_shrink_step;
            current_shrink_dist += global_shrink_step;

            int n_mx = static_cast<int>((new_x - map_origin_x) / res);
            int n_my = static_cast<int>((new_y - map_origin_y) / res);

            if (n_mx < 0 || n_mx >= width || n_my < 0 || n_my >= height) {
                ROS_INFO("[SHRINK] Shrunk point (%.2f, %.2f) out of bounds.", new_x, new_y);
                break;
            }

            // 【修复】检查 3x3 像素区域
            bool n_is_safe = true;
            int n_center_idx = n_my * width + n_mx;
            unsigned char n_center_cost = static_cast<unsigned char>(global_costmap.data[n_center_idx]);
            
            for (int dy = -safe_window_pixels; dy <= safe_window_pixels && n_is_safe; ++dy) {
                for (int dx = -safe_window_pixels; dx <= safe_window_pixels && n_is_safe; ++dx) {
                    int nx = n_mx + dx;
                    int ny = n_my + dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                        n_is_safe = false;
                        break;
                    }
                    int index = ny * width + nx;
                    if (index >= global_costmap.data.size()) {
                        n_is_safe = false;
                        break;
                    }
                    unsigned char cost = static_cast<unsigned char>(global_costmap.data[index]);
                    if (cost >= global_safe_threshold) {
                        n_is_safe = false;
                        break;
                    }
                }
            }

            if (n_is_safe) {
                ROS_INFO("[SHRINK] Found safe point at (%.2f, %.2f), dist_shrunk=%.3f, center_cost=%d",
                         new_x, new_y, current_shrink_dist, (int)n_center_cost);
                
                // 【新增】找到安全点后，继续额外收缩一段距离
                double extra_x = new_x;
                double extra_y = new_y;
                double extra_dist = 0.0;
                bool extra_safe = true;  // 【修复】标记额外收缩是否成功
                
                while (extra_dist < global_extra_shrink_distance) {
                    extra_x += dir_x * global_shrink_step;
                    extra_y += dir_y * global_shrink_step;
                    extra_dist += global_shrink_step;

                    // 检查额外收缩点是否仍然安全（简化检查，只检查单点）
                    int e_mx = static_cast<int>((extra_x - map_origin_x) / res);
                    int e_my = static_cast<int>((extra_y - map_origin_y) / res);
                    if (e_mx < 0 || e_mx >= width || e_my < 0 || e_my >= height) {
                        extra_safe = false;
                        break;
                    }
                    int e_index = e_my * width + e_mx;
                    if (e_index < global_costmap.data.size()) {
                        unsigned char e_cost = static_cast<unsigned char>(global_costmap.data[e_index]);
                        if (e_cost >= global_safe_threshold) {
                            ROS_INFO("[SHRINK] Extra point unsafe (cost=%d), using original safe point.", (int)e_cost);
                            extra_safe = false;  // 【修复】标记额外收缩失败
                            break;  // 额外点不安全，使用原安全点
                        }
                    }
                }

                // 使用额外收缩后的点（只有当额外收缩完全成功时）
                if (extra_safe && extra_dist >= global_extra_shrink_distance) {
                    target_x = extra_x;
                    target_y = extra_y;
                    ROS_INFO("[SHRINK] Using extra shrunk point (%.2f, %.2f).", target_x, target_y);
                } else {
                    target_x = new_x;
                    target_y = new_y;
                    ROS_INFO("[SHRINK] Using original safe point (%.2f, %.2f).", target_x, target_y);
                }
                found_safe = true;
                break;
            }
        }

        if (!found_safe) {
            ROS_WARN("[SHRINK] Failed to find safe point after shrinking %.2fm (max_radius=%.2fm).",
                     current_shrink_dist, global_shrink_radius);
        }

        return found_safe;
    }

    bool isGlobalCostValid(double x, double y)
    {
        if (!has_global_costmap || global_costmap.data.empty()) return false;
        double cost = getCost(x, y);
        return (cost < rescue_max_global_cost);
    }

    bool checkNearbyObstacleRunning()
    {
        if (!has_global_costmap || global_costmap.data.empty()) return false;
        
        double tx = current_goal.target_pose.pose.position.x;
        double ty = current_goal.target_pose.pose.position.y;
        
        double rx, ry, ryaw;
        if(!getRobotPose(rx, ry, ryaw)) return false;

        double dist = hypot(tx - rx, ty - ry);
        if (dist > nearby_check_radius) {
            return false;
        }

        double res = global_costmap.info.resolution;
        if (res <= 1e-6) return false;
        
        double map_origin_x = global_costmap.info.origin.position.x;
        double map_origin_y = global_costmap.info.origin.position.y;
        int width = global_costmap.info.width;
        int height = global_costmap.info.height;

        int mx = static_cast<int>((tx - map_origin_x) / res);
        int my = static_cast<int>((ty - map_origin_y) / res);

        if (mx < 0 || mx >= width || my < 0 || my >= height) return false;
        
        int index = my * width + mx;
        if (index >= global_costmap.data.size()) return false;

        int8_t raw = global_costmap.data[index];
        unsigned char cost = (raw < 0) ? 255 : static_cast<unsigned char>(raw);

        return (cost >= nearby_obstacle_threshold);
    }

    bool isNearbyCostSafe(double x, double y)
    {
        if (!has_global_costmap || global_costmap.data.empty()) return true; 
        
        double rx, ry, ryaw;
        if(!getRobotPose(rx, ry, ryaw)) return true;

        double dist = hypot(x - rx, y - ry);
        if (dist > nearby_check_radius) {
            return true;
        }

        double res = global_costmap.info.resolution;
        if (res <= 1e-6) return true;
        
        double map_origin_x = global_costmap.info.origin.position.x;
        double map_origin_y = global_costmap.info.origin.position.y;
        int width = global_costmap.info.width;
        int height = global_costmap.info.height;

        int mx = static_cast<int>((x - map_origin_x) / res);
        int my = static_cast<int>((y - map_origin_y) / res);

        if (mx < 0 || mx >= width || my < 0 || my >= height) return false;
        
        int index = my * width + mx;
        if (index >= global_costmap.data.size()) return false;

        int8_t raw = global_costmap.data[index];
        unsigned char cost = (raw < 0) ? 255 : static_cast<unsigned char>(raw);

        return (cost < nearby_obstacle_threshold);
    }

    double getCost(double x, double y)
    {
        if (!has_global_costmap || global_costmap.data.empty()) return 255;
        double res = global_costmap.info.resolution;
        if (res <= 1e-6) return 255;
        int mx = static_cast<int>((x - global_costmap.info.origin.position.x) / res);
        int my = static_cast<int>((y - global_costmap.info.origin.position.y) / res);
        int w = global_costmap.info.width;
        int h = global_costmap.info.height;
        if (mx < 0 || my < 0 || mx >= w || my >= h) return 255;
        int8_t raw = global_costmap.data[my * w + mx];
        return (raw < 0) ? 255.0 : static_cast<double>(raw);  // -1 = NO_INFORMATION (255)
    }

    double calculateUnknownProportion(double x, double y) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        
        if (!has_raw_map_ || !current_map_ || current_map_->data.empty()) {
            return 0.0; 
        }

        const auto& map = *current_map_;
        double res = map.info.resolution;
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;

        if (res <= 1e-6) return 0.0;

        int cx = static_cast<int>((x - origin_x) / res);
        int cy = static_cast<int>((y - origin_y) / res);
        
        if (cx < 0 || cx >= map.info.width || cy < 0 || cy >= map.info.height) {
            return 0.0;
        }

        int radius_cells = std::max(1, static_cast<int>(known_area_search_radius / res));
        radius_cells = std::min(radius_cells, 50); 

        int min_x = std::max(0, cx - radius_cells);
        int max_x = std::min((int)map.info.width - 1, cx + radius_cells);
        int min_y = std::max(0, cy - radius_cells);
        int max_y = std::min((int)map.info.height - 1, cy + radius_cells);

        int total = 0; 
        int unknown_count = 0; 
        
        double r_sq = known_area_search_radius * known_area_search_radius;
        
        for (int my = min_y; my <= max_y; ++my) {
            for (int mx = min_x; mx <= max_x; ++mx) {
                double wx = (mx + 0.5) * res + origin_x;
                double wy = (my + 0.5) * res + origin_y;

                if ((wx-x)*(wx-x) + (wy-y)*(wy-y) > r_sq) continue;

                total++;
                
                int8_t raw = map.data[my * map.info.width + mx];
                int val = static_cast<int>(raw);

                if (val == -1) {
                    unknown_count++;
                }
            }
        }

        if (total == 0) return 0.0;
        double prop = static_cast<double>(unknown_count) / total;
        if (prop > 1.0) prop = 1.0;
        if (prop < 0.0) prop = 0.0;
        return prop;
    }

    std::vector<FrontierCluster> clusterFrontiers()
    {
        std::vector<FrontierCluster> clusters;
        for(const auto &f : frontier_buffer) {
            bool merged = false;
            for(auto &c : clusters) {
                if(hypot(c.x - f.point.x, c.y - f.point.y) < cluster_radius) {
                    c.x = (c.x * c.num + f.point.x) / (c.num + 1);
                    c.y = (c.y * c.num + f.point.y) / (c.num + 1);
                    c.num++;
                    merged = true; break;
                }
            }
            if(!merged) {
                FrontierCluster c; c.x = f.point.x; c.y = f.point.y; c.num = 1;
                clusters.push_back(c);
            }
        }
        return clusters;
    }

    double calculateFrontierOrientation(double x, double y, double rx, double ry)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!has_raw_map_ || !current_map_ || current_map_->data.empty()) {
            return atan2(y - ry, x - rx);
        }
        const auto& map = *current_map_;
        double res = map.info.resolution;
        if (res <= 1e-6) return atan2(y - ry, x - rx);
        double origin_x = map.info.origin.position.x;
        double origin_y = map.info.origin.position.y;
        int width = map.info.width;
        int height = map.info.height;
        int cx = static_cast<int>((x - origin_x) / res);
        int cy = static_cast<int>((y - origin_y) / res);
        if (cx < 0 || cx >= width || cy < 0 || cy >= height) return atan2(y - ry, x - rx);
        int radius_cells = std::max(1, static_cast<int>(orientation_search_radius / res));
        radius_cells = std::min(radius_cells, 50); 
        int min_x = std::max(0, cx - radius_cells);
        int max_x = std::min(width - 1, cx + radius_cells);
        int min_y = std::max(0, cy - radius_cells);
        int max_y = std::min(height - 1, cy + radius_cells);
        double sum_dx = 0.0;
        double sum_dy = 0.0;
        int count = 0;
        int dx_nbr[4] = {1, -1, 0, 0};
        int dy_nbr[4] = {0, 0, 1, -1};
        for (int my = min_y; my <= max_y; ++my) {
            for (int mx = min_x; mx <= max_x; ++mx) {
                int index = my * width + mx;
                int8_t cell_val = map.data[index];
                if (cell_val == 0) { 
                    bool is_frontier_cell = false;
                    double local_fx = 0.0;
                    double local_fy = 0.0;
                    for (int i = 0; i < 4; ++i) {
                        int nx = mx + dx_nbr[i];
                        int ny = my + dy_nbr[i];
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int8_t neighbor_val = map.data[ny * width + nx];
                            if (neighbor_val == -1) { 
                                is_frontier_cell = true;
                                local_fx += dx_nbr[i];
                                local_fy += dy_nbr[i];
                            }
                        }
                    }
                    if (is_frontier_cell) {
                        double mag = sqrt(local_fx * local_fx + local_fy * local_fy);
                        if (mag > 1e-6) {
                            sum_dx += local_fx / mag;
                            sum_dy += local_fy / mag;
                            count++;
                        }
                    }
                }
            }
        }
        if (count > 0) {
            double avg_dx = sum_dx / count;
            double avg_dy = sum_dy / count;
            double mag = sqrt(avg_dx * avg_dx + avg_dy * avg_dy);
            if (mag > 1e-6) return atan2(avg_dy, avg_dx);
        }
        return atan2(y - ry, x - rx);
    }

    FrontierCluster selectGoal(bool force_revive)
    {
        double rx, ry, ryaw;
        if (!getRobotPose(rx, ry, ryaw)) return FrontierCluster();

        ros::Time now = ros::Time::now();
        failed_frontiers.erase(
            std::remove_if(failed_frontiers.begin(), failed_frontiers.end(),
                [&](const FailedFrontier& f){ return (now - f.stamp).toSec() > failed_frontier_timeout; }),
            failed_frontiers.end()
        );

        auto clusters = clusterFrontiers();
        if (clusters.empty()) return FrontierCluster();

        FrontierCluster best;
        const double MAX_DIST = 10.0;
        int max_num = 1;
        for(const auto& c : clusters) if(c.num > max_num) max_num = c.num;

        double min_input = std::max(-1.0, -expected_unknow_proportion);
        double max_input = std::min(1.0, 1.0 - expected_unknow_proportion);
        double max_arcsin_abs = std::max(std::abs(std::asin(min_input)), std::abs(std::asin(max_input)));
        if (max_arcsin_abs < 1e-6) max_arcsin_abs = 1e-6;

        std_msgs::Float32MultiArray debug_msg;
        debug_msg.data.clear();
        int point_id = 0;

        int total_points = clusters.size();
        int shrink_fail = 0, obstacle_fail = 0, blacklist_fail = 0, valid_count = 0;

        for (auto& c : clusters)
        {
            double orig_x = c.x, orig_y = c.y;  // 【调试】保存原始坐标
            double dist_to_robot = hypot(c.x - rx, c.y - ry);
            if (dist_to_robot < arrived_ignore_radius) continue;

            if (!shrinkOnGlobalCostmap(c.x, c.y, rx, ry)) {
                shrink_fail++;
                continue;
            }

            // 【调试】显示 shrink 效果
            if (std::abs(orig_x - c.x) > 0.01 || std::abs(orig_y - c.y) > 0.01) {
                ROS_INFO("[SHRINK_DEBUG] Point moved: (%.2f,%.2f) -> (%.2f,%.2f), dist=%.3fm",
                         orig_x, orig_y, c.x, c.y, hypot(orig_x - c.x, orig_y - c.y));
            }

            if (!isNearbyCostSafe(c.x, c.y)) {
                obstacle_fail++;
                continue;
            }

            bool blacklisted = false;
            for (const auto& f : failed_frontiers) {
                if (now < f.cool_until && hypot(c.x - f.x, c.y - f.y) < failed_frontier_tolerance * 2.0) {
                    blacklisted = true; break;
                }
                if (hypot(c.x - f.x, c.y - f.y) < failed_frontier_tolerance) {
                    blacklisted = true; break;
                }
            }
            if (!force_revive && blacklisted) {
                blacklist_fail++;
                continue;
            }

            valid_count++;
            c.distance = dist_to_robot;
            c.cost = getCost(c.x, c.y);

            double raw_num = double(c.num) / max_num; 
            double raw_dist_norm = c.distance / MAX_DIST; 
            double actual_unknown_prop = calculateUnknownProportion(c.x, c.y);
            double input_val = std::max(-1.0, std::min(1.0, actual_unknown_prop - expected_unknow_proportion));
            double raw_area_val = std::asin(input_val) / max_arcsin_abs; 

            double term_num = raw_num * num_weight;
            double term_dist = raw_dist_norm * distance_weight; 
            double term_area = raw_area_val * known_area_punishment;

            if (force_revive) term_dist *= 1.5;

            c.weight = term_num + term_dist + term_area;

            debug_msg.data.push_back((float)point_id);
            debug_msg.data.push_back((float)c.x);
            debug_msg.data.push_back((float)c.y);
            debug_msg.data.push_back((float)raw_num);
            debug_msg.data.push_back((float)raw_dist_norm);
            debug_msg.data.push_back((float)raw_area_val);
            debug_msg.data.push_back((float)c.weight);

            if (c.weight > best.weight) best = c;
            point_id++;
        }

        // 【调试】输出选点统计
        ros::Time select_time = ros::Time::now();
        ROS_INFO("[GOAL_SELECT] Total=%d, shrink_fail=%d, obstacle_fail=%d, blacklist_fail=%d, valid=%d, best_weight=%.2f, costmap_stamp=%d.%09d",
                 total_points, shrink_fail, obstacle_fail, blacklist_fail, valid_count, best.weight, 
                 global_costmap.header.stamp.sec, global_costmap.header.stamp.nsec);

        candidate_pub.publish(debug_msg);
        return best;
    }

    // 【修复版】attemptRescueGoal
    // 1. 在救援半径内随机选点（不是低代价区）
    // 2. 救援点不参与失败计数
    bool attemptRescueGoal()
    {
        if (frontier_buffer.empty()) return false;
        if (!has_global_costmap) return false;
        
        double rx, ry, ryaw;
        if (!getRobotPose(rx, ry, ryaw)) return false;

        // 【修复】只在第一次进入救援模式时计算救援中心
        if (rescue_attempt_count == 0) {
            double sum_x = 0, sum_y = 0;
            for (const auto& p : frontier_buffer) {
                sum_x += p.point.x;
                sum_y += p.point.y;
            }
            rescue_center_x = sum_x / frontier_buffer.size();
            rescue_center_y = sum_y / frontier_buffer.size();
            ROS_WARN_STREAM(">>> RESCUE MODE ACTIVATED <<< Center: (" << rescue_center_x << ", " << rescue_center_y << ")");
            publishRescueInfo();  // 【新增】发布救援信息
        }

        // 【修复】在救援半径内随机选点
        std::uniform_real_distribution<> dist_angle(0, 2 * 3.14159);
        std::uniform_real_distribution<> dist_radius(0.5, rescue_search_radius);  // 最小 0.5 米，避免太近
        std::uniform_real_distribution<> dist_yaw(-3.14159, 3.14159);

        int max_tries = 10;
        for (int i = 0; i < max_tries; ++i) {
            double angle = dist_angle(gen);
            double radius = dist_radius(gen);
            
            double rand_x = rescue_center_x + radius * cos(angle);
            double rand_y = rescue_center_y + radius * sin(angle);

            // 【修复】只检查是否在救援半径内，不检查距离机器人太近
            if (hypot(rand_x - rescue_center_x, rand_y - rescue_center_y) > rescue_search_radius) continue;
            
            // 【修复】救援点只需要是有效的全局代价，不要求是低代价区
            if (!isGlobalCostValid(rand_x, rand_y)) continue;
            
            // 【修复】shrink 检查
            if (!shrinkOnGlobalCostmap(rand_x, rand_y, rx, ry)) continue;
            
            // 【带距离门控的】救援点检查
            if (!isNearbyCostSafe(rand_x, rand_y)) continue;

            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = rand_x;
            goal.target_pose.pose.position.y = rand_y;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(dist_yaw(gen));

            ROS_WARN_STREAM("Rescue Attempt #" << (rescue_attempt_count + 1) << ": Sending random goal (" << rand_x << ", " << rand_y << ")");
            
            ac.sendGoal(goal);
            current_goal = goal;
            goal_active = true;
            cancel_pending = false;
            just_canceled = false;
            plan_received = false;
            waiting_for_plan = true;
            goal_send_time = ros::Time::now();
            goal_start_time = ros::Time::now(); 
            
            rescue_attempt_count++;
            last_obstacle_check_time = ros::Time::now();
            obstacle_suspected = false;
            distance_initialized = false; 
            last_distance = 0;
            last_distance_time = ros::Time::now();
            
            publishRescueInfo();  // 【新增】每次尝试后发布
            return true;
        }

        ROS_WARN("Rescue: Could not find valid random point.");
        publishRescueInfo();  // 【新增】失败也发布
        return false;
    }

    void sendGoal(const FrontierCluster &c)
    {
        double rx, ry, ryaw;
        if(!getRobotPose(rx, ry, ryaw)) return;

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = c.x;
        goal.target_pose.pose.position.y = c.y;

        double yaw = calculateFrontierOrientation(c.x, c.y, rx, ry);
        if (std::isnan(yaw)) yaw = atan2(c.y - ry, c.x - rx);
        if (std::isnan(yaw)) yaw = ryaw;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        ac.sendGoal(goal);
        current_goal = goal;
        goal_active = true;
        cancel_pending = false;
        just_canceled = false;
        is_rescue_mode = false;
        plan_received = false;
        waiting_for_plan = true;
        goal_send_time = ros::Time::now();

        last_obstacle_check_time = ros::Time::now();
        obstacle_suspected = false;
        distance_initialized = false;
        last_distance = 0;
        last_distance_time = ros::Time::now();
    }

    void requestCancel()
    {
        if (!goal_active) return;
        ROS_WARN("Requesting Cancel...");
        ac.cancelAllGoals();
        cancel_pending = true;
        cancel_request_time = ros::Time::now();
    }

    void finalizeCancel()
    {
        ROS_INFO("Finalizing Cancel. consecutive_failures=%d", consecutive_plan_failures);
        // 【修复】救援模式的 goal 不参与失败计数
        if (!is_rescue_mode) {
            failed_frontiers.emplace_back(
                current_goal.target_pose.pose.position.x,
                current_goal.target_pose.pose.position.y,
                ros::Time::now(),
                ros::Time::now() + ros::Duration(failed_frontier_cool_down)
            );
        } else {
            ROS_WARN("Rescue attempt failed/cancelled.");
        }
        goal_active = false;
        cancel_pending = false;
        just_canceled = true;
        slow_count = 0;
        plan_received = false;
        waiting_for_plan = false;
        obstacle_suspected = false;
        distance_initialized = false;
        publishRescueInfo();  // 【新增】发布更新
    }

    // 【修复版】monitorGoal
    // 1. 救援模式也使用 goal monitor 监控
    // 2. 增加节点内部到达判断（0.5 米内算成功）
    // 3. 救援模式使用握手确认方式退出
    void monitorGoal()
    {
        if(!goal_active) return;
        actionlib::SimpleClientGoalState state = ac.getState();
        
        if (cancel_pending) {
            if (state.isDone()) {
                ROS_INFO_STREAM("Cancel Confirmed: " << state.toString());
                finalizeCancel();
                return;
            }
            if ((ros::Time::now() - cancel_request_time).toSec() > cancel_timeout) {
                ROS_ERROR("Cancel Timeout! Force reset.");
                finalizeCancel();
                return;
            }
            return;
        }

        if (state == actionlib::SimpleClientGoalState::ABORTED ||
            state == actionlib::SimpleClientGoalState::LOST) {
            if (is_rescue_mode) {
                ROS_WARN("Rescue Goal Aborted/Lost.");
                finalizeCancel();
                return;
            } else {
                ROS_WARN_STREAM("Goal Aborted/Lost: " << state.getText());
                failed_frontiers.emplace_back(
                    current_goal.target_pose.pose.position.x,
                    current_goal.target_pose.pose.position.y,
                    ros::Time::now(),
                    ros::Time::now() + ros::Duration(failed_frontier_cool_down)
                );
                consecutive_plan_failures++;  // 【新增】连续 failed 计数
                ROS_WARN_STREAM("Consecutive plan failures: " << consecutive_plan_failures);
                finalizeCancel();
                return;
            }
        }

        // 【修复】救援模式也使用 plan timeout 检测（握手确认方式）
        double current_plan_timeout = is_rescue_mode ? rescue_plan_timeout : plan_timeout;
        if(waiting_for_plan && !plan_received) {
            if((ros::Time::now() - goal_send_time).toSec() > current_plan_timeout) {
                ROS_WARN(is_rescue_mode ? "Rescue Plan Timeout" : "Plan Timeout");
                // 【修复】救援模式超时不加入黑名单
                if (!is_rescue_mode) {
                    failed_frontiers.emplace_back(
                        current_goal.target_pose.pose.position.x,
                        current_goal.target_pose.pose.position.y,
                        ros::Time::now(),
                        ros::Time::now() + ros::Duration(failed_frontier_cool_down)
                    );
                    consecutive_plan_failures++;  // 【新增】连续 failed 计数
                    ROS_WARN_STREAM("Consecutive plan failures: " << consecutive_plan_failures);
                    requestCancel();
                    return;
                } else {
                    ROS_WARN("Rescue plan timeout, requesting cancel...");
                    requestCancel();
                    return;
                }
            }
            return;
        }
        if(!plan_received) return;

        // 【修复】救援模式也需要监控
        double rx, ry, ryaw;
        if(getRobotPose(rx, ry, ryaw)) {
            double dx = current_goal.target_pose.pose.position.x - rx;
            double dy = current_goal.target_pose.pose.position.y - ry;
            double dist = hypot(dx, dy);
            
            // 【新增】救援模式内部到达判断（0.5 米内算成功）
            if (is_rescue_mode && dist <= 0.5) {
                ROS_INFO_STREAM("Rescue internal arrival detected within 0.5m (dist=" << dist << "). Requesting cancel for handshake-confirmed exit.");
                // 【修复】使用握手确认方式退出
                if (!just_canceled) {
                    requestCancel();
                }
                return;
            }
            
            // 正常模式的停滞检测
            if (!is_rescue_mode) {
                if (distance_initialized && (ros::Time::now()-last_distance_time).toSec() > goal_check_interval) {
                    if (last_distance - dist < distance_change_threshold) {
                        slow_count++;
                        if(slow_count >= slow_count_limit) {
                            ROS_WARN("Robot Stuck (No movement)");
                            failed_frontiers.emplace_back(
                                current_goal.target_pose.pose.position.x,
                                current_goal.target_pose.pose.position.y,
                                ros::Time::now(),
                                ros::Time::now() + ros::Duration(failed_frontier_cool_down)
                            );
                            requestCancel();
                            return;
                        }
                    } else { slow_count = 0; }
                    last_distance = dist;
                    last_distance_time = ros::Time::now();
                } else if (!distance_initialized) {
                    last_distance = dist;
                    last_distance_time = ros::Time::now();
                    distance_initialized = true;
                }

                if((ros::Time::now()-goal_start_time).toSec() > abort_timeout) {
                    ROS_WARN("Goal Total Timeout");
                    failed_frontiers.emplace_back(
                        current_goal.target_pose.pose.position.x,
                        current_goal.target_pose.pose.position.y,
                        ros::Time::now(),
                        ros::Time::now() + ros::Duration(failed_frontier_cool_down)
                    );
                    requestCancel();
                    return;
                }

                if ((ros::Time::now() - last_obstacle_check_time).toSec() > obstacle_check_interval) {
                    last_obstacle_check_time = ros::Time::now();
                    if (checkNearbyObstacleRunning()) {
                        if (!obstacle_suspected) {
                            obstacle_suspected = true;
                            first_obstacle_detect_time = ros::Time::now();
                        } else if ((ros::Time::now() - first_obstacle_detect_time).toSec() > nearby_obstacle_confirm_delay) {
                            ROS_WARN("Obstacle Confirmed (Nearby High Cost) - Cancelling Goal");
                            requestCancel();
                        }
                    } else { obstacle_suspected = false; }
                }
            } else {
                // 【修复】救援模式也监控，但使用简化的超时检测
                if((ros::Time::now()-goal_start_time).toSec() > rescue_plan_timeout * 2.0) {
                    ROS_WARN("Rescue Goal Timeout");
                    requestCancel();
                    return;
                }
            }
        }

        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO(is_rescue_mode ? "Rescue Successful!" : "Goal Reached");
            // 【修复】救援模式成功不加入黑名单
            if (!is_rescue_mode) {
                failed_frontiers.emplace_back(
                    current_goal.target_pose.pose.position.x,
                    current_goal.target_pose.pose.position.y,
                    ros::Time::now(),
                    ros::Time::now() + ros::Duration(failed_frontier_cool_down * 2.0)
                );
            }
            goal_active = false;
            plan_received = false;
            waiting_for_plan = false;
            distance_initialized = false;
            consecutive_plan_failures = 0;  // 【新增】成功时重置连续 failed 计数

            // 【修复】只有在完成一次救援后才退出救援模式
            if (is_rescue_mode) {
                ROS_WARN("Rescue goal completed. Exiting rescue mode.");
                is_rescue_mode = false;
                rescue_attempt_count = 0;
                rescue_center_x = 0.0;
                rescue_center_y = 0.0;
                publishRescueInfo();  // 【新增】发布退出信息
            }
            return;
        }
    }

    void spin()
    {
        ros::Rate r(10);
        while(ros::ok()) {
            monitorGoal();
            if (just_canceled) {
                just_canceled = false;
                ros::spinOnce();
                r.sleep();
                continue;
            }

            if (!goal_active && !cancel_pending) {
                // 【修复】救援模式触发逻辑：基于连续 plan failed 次数，而不是"没有 goal 可选"
                if (!is_rescue_mode && consecutive_plan_failures >= rescue_trigger_failures) {
                    ROS_WARN_STREAM("Consecutive plan failures reached " << consecutive_plan_failures
                        << ". Activating ULTIMATE RESCUE MODE.");
                    is_rescue_mode = true;
                    rescue_attempt_count = 0;
                    rescue_start_time = ros::Time::now();
                    // 计算救援中心
                    double sum_x = 0, sum_y = 0;
                    for (const auto& p : frontier_buffer) {
                        sum_x += p.point.x;
                        sum_y += p.point.y;
                    }
                    rescue_center_x = sum_x / frontier_buffer.size();
                    rescue_center_y = sum_y / frontier_buffer.size();
                    ROS_WARN_STREAM(">>> RESCUE MODE ACTIVATED <<< Center: (" << rescue_center_x << ", " << rescue_center_y << ")");
                    publishRescueInfo();  // 【新增】发布进入救援信息
                    if (!attemptRescueGoal()) {
                        ROS_WARN("Rescue Mode Active but failed to generate initial point. Will retry next cycle.");
                    }
                    ros::spinOnce();
                    r.sleep();
                    continue;
                }

                bool sent = false;
                if (!frontier_buffer.empty()) {
                    FrontierCluster goal = selectGoal(false);
                    ROS_INFO("[SPIN] selectGoal returned weight=%.2f, consecutive_failures=%d", goal.weight, consecutive_plan_failures);
                    if (goal.weight > -1e8) {
                        sendGoal(goal);
                        sent = true;
                    }
                }
                if (!sent && !frontier_buffer.empty()) {
                     FrontierCluster revived = selectGoal(true);
                     ROS_INFO("[SPIN] selectGoal(revive) returned weight=%.2f", revived.weight);
                     if (revived.weight > -1e8) {
                        sendGoal(revived);
                        sent = true;
                     }
                }

                if (!sent && !frontier_buffer.empty()) {
                    ROS_WARN("[SPIN] No valid goal found. frontier_buffer size=%zu", frontier_buffer.size());
                }

                // 救援模式重新选点逻辑
                if (is_rescue_mode) {
                    if (rescue_attempt_count >= rescue_max_attempts) {
                        ROS_ERROR("Rescue Max Attempts Reached (%d). Exiting Rescue Mode.", rescue_max_attempts);
                        is_rescue_mode = false;
                        rescue_attempt_count = 0;
                        rescue_center_x = 0.0;
                        rescue_center_y = 0.0;
                        ros::Time now = ros::Time::now();
                        size_t before = failed_frontiers.size();
                        failed_frontiers.erase(
                            std::remove_if(failed_frontiers.begin(), failed_frontiers.end(),
                                [&](const FailedFrontier& f){ return (now - f.stamp).toSec() > failed_frontier_timeout; }),
                            failed_frontiers.end()
                        );
                        ROS_INFO("Cleaned %zu expired failed frontiers.", before - failed_frontiers.size());
                        publishRescueInfo();  // 【新增】发布退出信息
                    } else {
                        if (!attemptRescueGoal()) {
                            ROS_WARN_THROTTLE(2.0, "Rescue: Failed to generate point (Attempt %d/%d). Retrying...",
                                              rescue_attempt_count, rescue_max_attempts);
                        }
                    }
                }
            }
            ros::spinOnce();
            r.sleep();
        }
    }
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"rrt_goal_decision");
    FrontierGoalManager manager;
    manager.spin();
    return 0;
}
