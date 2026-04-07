#include "rrt_goal_decision/frontier_goal_manager.h"
#include "rrt_goal_decision/frontier_utils.h"
#include "rrt_goal_decision/frontier_params.h"

FrontierGoalManager::FrontierGoalManager() : pnh("~"), ac("move_base", true), gen(std::random_device{}())
{
    // 加载参数
    params = frontier_params::loadParams(pnh);
    frontier_params::printParams(params);

    has_global_costmap = false;
    has_raw_map_ = false;

    frontier_sub = nh.subscribe("/detected_points", 100, &FrontierGoalManager::frontierCallback, this);
    global_costmap_sub = nh.subscribe("/move_base/global_costmap/costmap", 10, &FrontierGoalManager::globalCostmapCallback, this);
    costmap_updates_sub = nh.subscribe("/move_base/global_costmap/costmap_updates", 10, &FrontierGoalManager::costmapUpdatesCallback, this);
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
    consecutive_plan_failures = 0;
    rescue_trigger_count = 0;
    rescue_goal_start_time = ros::Time(0);
    last_stuck_abort_check_time = ros::Time(0);
    last_robot_position.x = 0.0;
    last_robot_position.y = 0.0;
    rescue_mode_start_time = ros::Time(0);
    rescue_mode_total_duration = 0.0;

    ROS_INFO("Waiting for move_base...");
    if (!ac.waitForServer(ros::Duration(10.0))) {
        ROS_ERROR("move_base server not found.");
        ros::shutdown();
        return;
    }

    publishRescueInfo();
    ROS_INFO("Frontier Explorer Ready (V5.7 - Smart Rescue with Displacement Check).");
}

void FrontierGoalManager::publishRescueInfo()
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = is_rescue_mode ? 1.0 : 0.0;
    msg.data[1] = rescue_center_x;
    msg.data[2] = rescue_center_y;
    msg.data[3] = static_cast<float>(rescue_attempt_count);
    rescue_info_pub.publish(msg);
}

void FrontierGoalManager::frontierCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (frontier_buffer.size() > 2000) frontier_buffer.erase(frontier_buffer.begin());
    frontier_buffer.push_back(*msg);
}

void FrontierGoalManager::globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    global_costmap = *msg;
    has_global_costmap = true;
    ROS_INFO("[COSTMAP] Received initial global costmap: %dx%d, resolution=%.3f",
             msg->info.width, msg->info.height, msg->info.resolution);
}

void FrontierGoalManager::costmapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
    if (!has_global_costmap)
    {
        ROS_WARN("[COSTMAP_UPDATE] Received update before initial map!");
        return;
    }

    if (msg->x < 0 || msg->y < 0 ||
        msg->x + msg->width > global_costmap.info.width ||
        msg->y + msg->height > global_costmap.info.height)
    {
        ROS_ERROR("[COSTMAP_UPDATE] Update out of bounds!");
        return;
    }

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

    ROS_INFO_THROTTLE(10.0, "[COSTMAP_UPDATE] Applied %d updates at (%d,%d) size=%ux%u",
                     updates, msg->x, msg->y, msg->width, msg->height);
}

void FrontierGoalManager::rawMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    current_map_ = msg;
    has_raw_map_ = true;
}

void FrontierGoalManager::planCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (!goal_active || plan_received || msg->poses.empty()) return;
    plan_received = true;
    waiting_for_plan = false;
    if (!is_rescue_mode) {
        goal_start_time = ros::Time::now();
    }
    slow_count = 0;
}

bool FrontierGoalManager::getRobotPose(double &x, double &y, double &yaw)
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

bool FrontierGoalManager::shrinkOnGlobalCostmap(double &target_x, double &target_y, double robot_x, double robot_y)
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

    int safe_window_pixels = params.global_safe_window_pixels;

    int mx = static_cast<int>((target_x - map_origin_x) / res);
    int my = static_cast<int>((target_y - map_origin_y) / res);

    if (mx < 0 || mx >= width || my < 0 || my >= height) {
        return false;
    }

    int center_index = my * width + mx;
    int8_t raw_center = global_costmap.data[center_index];
    unsigned char center_cost = (raw_center < 0) ? 255 : static_cast<unsigned char>(raw_center);

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
            if (cost >= params.global_safe_threshold) {
                is_safe = false;
                unsafe_count++;
            }
        }
    }

    if (is_safe) {
        return true;
    }

    double dx = robot_x - target_x;
    double dy = robot_y - target_y;
    double dist = hypot(dx, dy);

    double dir_x = dx / dist;
    double dir_y = dy / dist;

    double current_shrink_dist = 0.0;
    double new_x = target_x;
    double new_y = target_y;
    bool found_safe = false;

    while (current_shrink_dist < params.global_shrink_radius) {
        new_x += dir_x * params.global_shrink_step;
        new_y += dir_y * params.global_shrink_step;
        current_shrink_dist += params.global_shrink_step;

        int n_mx = static_cast<int>((new_x - map_origin_x) / res);
        int n_my = static_cast<int>((new_y - map_origin_y) / res);

        if (n_mx < 0 || n_mx >= width || n_my < 0 || n_my >= height) {
            break;
        }

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
                if (cost >= params.global_safe_threshold) {
                    n_is_safe = false;
                    break;
                }
            }
        }

        if (n_is_safe) {
            double extra_x = new_x;
            double extra_y = new_y;
            double extra_dist = 0.0;
            bool extra_safe = true;

            while (extra_dist < params.global_extra_shrink_distance) {
                extra_x += dir_x * params.global_shrink_step;
                extra_y += dir_y * params.global_shrink_step;
                extra_dist += params.global_shrink_step;

                int e_mx = static_cast<int>((extra_x - map_origin_x) / res);
                int e_my = static_cast<int>((extra_y - map_origin_y) / res);
                if (e_mx < 0 || e_mx >= width || e_my < 0 || e_my >= height) {
                    extra_safe = false;
                    break;
                }
                int e_index = e_my * width + e_mx;
                if (e_index < global_costmap.data.size()) {
                    unsigned char e_cost = static_cast<unsigned char>(global_costmap.data[e_index]);
                    if (e_cost >= params.global_safe_threshold) {
                        extra_safe = false;
                        break;
                    }
                }
            }

            if (extra_safe && extra_dist >= params.global_extra_shrink_distance) {
                target_x = extra_x;
                target_y = extra_y;
            } else {
                target_x = new_x;
                target_y = new_y;
            }
            found_safe = true;
            break;
        }
    }

    return found_safe;
}

bool FrontierGoalManager::isGlobalCostValid(double x, double y)
{
    if (!has_global_costmap || global_costmap.data.empty()) return false;
    double cost = getCost(x, y);
    return (cost < params.rescue_max_global_cost);
}

bool FrontierGoalManager::checkNearbyObstacleRunning()
{
    if (!has_global_costmap || global_costmap.data.empty()) return false;

    double tx = current_goal.target_pose.pose.position.x;
    double ty = current_goal.target_pose.pose.position.y;

    double rx, ry, ryaw;
    if(!getRobotPose(rx, ry, ryaw)) return false;

    double dist = hypot(tx - rx, ty - ry);
    if (dist > params.nearby_check_radius) {
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

    return (cost >= params.nearby_obstacle_threshold);
}

bool FrontierGoalManager::isNearbyCostSafe(double x, double y)
{
    if (!has_global_costmap || global_costmap.data.empty()) return true;

    double rx, ry, ryaw;
    if(!getRobotPose(rx, ry, ryaw)) return true;

    double dist = hypot(x - rx, y - ry);
    if (dist > params.nearby_check_radius) {
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

    return (cost < params.nearby_obstacle_threshold);
}

double FrontierGoalManager::getCost(double x, double y)
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
    return (raw < 0) ? 255.0 : static_cast<double>(raw);
}

double FrontierGoalManager::calculateUnknownProportion(double x, double y) {
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

    int radius_cells = std::max(1, static_cast<int>(params.known_area_search_radius / res));
    radius_cells = std::min(radius_cells, 50);

    int min_x = std::max(0, cx - radius_cells);
    int max_x = std::min((int)map.info.width - 1, cx + radius_cells);
    int min_y = std::max(0, cy - radius_cells);
    int max_y = std::min((int)map.info.height - 1, cy + radius_cells);

    int total = 0;
    int unknown_count = 0;

    double r_sq = params.known_area_search_radius * params.known_area_search_radius;

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

std::vector<FrontierCluster> FrontierGoalManager::clusterFrontiers()
{
    std::vector<FrontierCluster> clusters;
    for(const auto &f : frontier_buffer) {
        bool merged = false;
        for(auto &c : clusters) {
            if(hypot(c.x - f.point.x, c.y - f.point.y) < params.cluster_radius) {
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

double FrontierGoalManager::calculateFrontierOrientation(double x, double y, double rx, double ry)
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
    int radius_cells = std::max(1, static_cast<int>(params.orientation_search_radius / res));
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

FrontierCluster FrontierGoalManager::selectGoal(bool force_revive)
{
    double rx, ry, ryaw;
    if (!getRobotPose(rx, ry, ryaw)) return FrontierCluster();

    ros::Time now = ros::Time::now();
    failed_frontiers.erase(
        std::remove_if(failed_frontiers.begin(), failed_frontiers.end(),
            [&](const FailedFrontier& f){ return now > f.cool_until; }),
        failed_frontiers.end()
    );

    auto clusters = clusterFrontiers();
    if (clusters.empty()) return FrontierCluster();

    FrontierCluster best;
    const double MAX_DIST = 10.0;
    int max_num = 1;
    for(const auto& c : clusters) if(c.num > max_num) max_num = c.num;

    double min_input = std::max(-1.0, -params.expected_unknow_proportion);
    double max_input = std::min(1.0, 1.0 - params.expected_unknow_proportion);
    double max_arcsin_abs = std::max(std::abs(std::asin(min_input)), std::abs(std::asin(max_input)));
    if (max_arcsin_abs < 1e-6) max_arcsin_abs = 1e-6;

    std_msgs::Float32MultiArray debug_msg;
    debug_msg.data.clear();
    int point_id = 0;

    int total_points = clusters.size();
    int shrink_fail = 0, obstacle_fail = 0, blacklist_fail = 0, valid_count = 0;

    for (auto& c : clusters)
    {
        double orig_x = c.x, orig_y = c.y;
        double dist_to_robot = hypot(c.x - rx, c.y - ry);
        if (dist_to_robot < params.arrived_ignore_radius) continue;

        if (!shrinkOnGlobalCostmap(c.x, c.y, rx, ry)) {
            shrink_fail++;
            continue;
        }

        if (!isNearbyCostSafe(c.x, c.y)) {
            obstacle_fail++;
            continue;
        }

        bool blacklisted = false;
        for (const auto& f : failed_frontiers) {
            if (hypot(c.x - f.x, c.y - f.y) < params.failed_frontier_tolerance) {
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
        double input_val = std::max(-1.0, std::min(1.0, actual_unknown_prop - params.expected_unknow_proportion));
        double raw_area_val = std::asin(input_val) / max_arcsin_abs;

        double term_num = raw_num * params.num_weight;
        double term_dist = raw_dist_norm * params.distance_weight;
        double term_area = raw_area_val * params.known_area_punishment;

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

    ros::Time select_time = ros::Time::now();
    ROS_INFO("[GOAL_SELECT] Total=%d, shrink_fail=%d, obstacle_fail=%d, blacklist_fail=%d, valid=%d, best_weight=%.2f, costmap_stamp=%d.%09d",
             total_points, shrink_fail, obstacle_fail, blacklist_fail, valid_count, best.weight,
             global_costmap.header.stamp.sec, global_costmap.header.stamp.nsec);

    candidate_pub.publish(debug_msg);
    return best;
}

bool FrontierGoalManager::attemptRescueGoal()
{
    if (frontier_buffer.empty()) return false;
    if (!has_global_costmap) return false;

    double rx, ry, ryaw;
    if (!getRobotPose(rx, ry, ryaw)) return false;

    if (rescue_attempt_count == 0) {
        double sum_x = 0, sum_y = 0;
        for (const auto& p : frontier_buffer) {
            sum_x += p.point.x;
            sum_y += p.point.y;
        }
        rescue_center_x = sum_x / frontier_buffer.size();
        rescue_center_y = sum_y / frontier_buffer.size();
        ROS_WARN_STREAM(">>> RESCUE MODE ACTIVATED <<< Center: (" << rescue_center_x << ", " << rescue_center_y << ")");
        publishRescueInfo();
    }

    std::uniform_real_distribution<> dist_angle(0, 2 * 3.14159);
    std::uniform_real_distribution<> dist_radius(0.5, params.rescue_search_radius);
    std::uniform_real_distribution<> dist_yaw(-3.14159, 3.14159);

    int max_tries = 10;
    for (int i = 0; i < max_tries; ++i) {
        double angle = dist_angle(gen);
        double radius = dist_radius(gen);

        double rand_x = rescue_center_x + radius * cos(angle);
        double rand_y = rescue_center_y + radius * sin(angle);

        if (hypot(rand_x - rescue_center_x, rand_y - rescue_center_y) > params.rescue_search_radius) continue;

        if (!isGlobalCostValid(rand_x, rand_y)) continue;

        if (!shrinkOnGlobalCostmap(rand_x, rand_y, rx, ry)) continue;

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

        rescue_goal_start_time = ros::Time::now();

        rescue_attempt_count++;
        last_obstacle_check_time = ros::Time::now();
        obstacle_suspected = false;
        distance_initialized = false;
        last_distance = 0;
        last_distance_time = ros::Time::now();

        publishRescueInfo();
        return true;
    }

    ROS_WARN("Rescue: Could not find valid random point.");
    publishRescueInfo();
    return false;
}

void FrontierGoalManager::sendGoal(const FrontierCluster &c)
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

void FrontierGoalManager::requestCancel()
{
    if (!goal_active) return;
    ROS_WARN("Requesting Cancel...");
    ac.cancelAllGoals();
    cancel_pending = true;
    cancel_request_time = ros::Time::now();
}

void FrontierGoalManager::finalizeCancel()
{
    ROS_INFO("Finalizing Cancel. consecutive_failures=%d", consecutive_plan_failures);
    if (!is_rescue_mode) {
        markGoalAsFailed(
            current_goal.target_pose.pose.position.x,
            current_goal.target_pose.pose.position.y
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
    first_obstacle_detect_time = ros::Time(0);
    distance_initialized = false;

    if (is_rescue_mode) {
        rescue_trigger_count = 0;
        consecutive_plan_failures = 0;
        last_stuck_abort_check_time = ros::Time(0);
        last_robot_position.x = 0.0;
        last_robot_position.y = 0.0;
        ROS_INFO("[Rescue Exit] All abort counters and positions reset.");
    }

    publishRescueInfo();
}

void FrontierGoalManager::markGoalAsFailed(double x, double y, double cool_down_multiplier)
{
    failed_frontiers.emplace_back(
        x,
        y,
        ros::Time::now(),
        ros::Time::now() + ros::Duration(params.failed_frontier_cool_down * cool_down_multiplier)
    );
}

void FrontierGoalManager::incrementRescueTriggerCount(const std::string& reason)
{
    double rx, ry, ryaw;
    if (!getRobotPose(rx, ry, ryaw)) return;

    if (!last_stuck_abort_check_time.isValid()) {
        last_robot_position.x = rx;
        last_robot_position.y = ry;
        last_stuck_abort_check_time = ros::Time::now();
        ROS_INFO("[Rescue Counter] First %s, recording position (%.2f, %.2f)", reason.c_str(), rx, ry);
    } else {
        if (frontier_utils::checkDisplacement(rx, ry, last_robot_position.x, last_robot_position.y, 0.5)) {
            ROS_INFO("[Rescue Counter] Robot moved >0.5m during %s, reset rescue_trigger_count to 0", reason.c_str());
            rescue_trigger_count = 0;
        } else {
            rescue_trigger_count++;
            ROS_INFO("[Rescue Counter] rescue_trigger_count = %d/%d (%s)",
                     rescue_trigger_count, params.rescue_trigger_count_threshold, reason.c_str());
        }
        last_stuck_abort_check_time = ros::Time::now();
    }
}

void FrontierGoalManager::monitorGoal()
{
    if(!goal_active) return;
    actionlib::SimpleClientGoalState state = ac.getState();

    if (cancel_pending) {
        if (state.isDone()) {
            ROS_INFO_STREAM("Cancel Confirmed: " << state.toString());
            finalizeCancel();
            return;
        }
        if ((ros::Time::now() - cancel_request_time).toSec() > params.cancel_timeout) {
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
                ros::Time::now() + ros::Duration(params.failed_frontier_cool_down)
            );
            consecutive_plan_failures++;
            ROS_WARN_STREAM("Consecutive plan failures: " << consecutive_plan_failures);

            incrementRescueTriggerCount("move_base ABORTED");

            finalizeCancel();
            return;
        }
    }

    double current_plan_timeout = is_rescue_mode ? params.rescue_plan_timeout : params.plan_timeout;
    if(waiting_for_plan && !plan_received) {
        if((ros::Time::now() - goal_send_time).toSec() > current_plan_timeout) {
            ROS_WARN(is_rescue_mode ? "Rescue Plan Timeout" : "Plan Timeout");
            if (!is_rescue_mode) {
                failed_frontiers.emplace_back(
                    current_goal.target_pose.pose.position.x,
                    current_goal.target_pose.pose.position.y,
                    ros::Time::now(),
                    ros::Time::now() + ros::Duration(params.failed_frontier_cool_down)
                );
                consecutive_plan_failures++;
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

    double rx, ry, ryaw;
    if(getRobotPose(rx, ry, ryaw)) {
        double dx = current_goal.target_pose.pose.position.x - rx;
        double dy = current_goal.target_pose.pose.position.y - ry;
        double dist = hypot(dx, dy);

        if (is_rescue_mode && dist <= 0.5) {
            ROS_INFO_STREAM("Rescue internal arrival detected within 0.5m (dist=" << dist << "). Requesting cancel for handshake-confirmed exit.");
            if (!just_canceled) {
                requestCancel();
            }
            return;
        }

        if (is_rescue_mode && rescue_goal_start_time.isValid()) {
            double goal_elapsed = (ros::Time::now() - rescue_goal_start_time).toSec();
            if (goal_elapsed > params.rescue_goal_timeout) {
                ROS_WARN("[Rescue Timeout] Goal timeout (%.1fs > %.1fs), trying new rescue point...",
                         goal_elapsed, params.rescue_goal_timeout);
                requestCancel();
                return;
            }
        }

        if (!is_rescue_mode) {
            if (distance_initialized && (ros::Time::now()-last_distance_time).toSec() > params.goal_check_interval) {
                if (last_distance - dist < params.distance_change_threshold) {
                    slow_count++;
                    if(slow_count >= params.slow_count_limit) {
                        ROS_WARN("Robot Stuck (No movement)");
                        failed_frontiers.emplace_back(
                            current_goal.target_pose.pose.position.x,
                            current_goal.target_pose.pose.position.y,
                            ros::Time::now(),
                            ros::Time::now() + ros::Duration(params.failed_frontier_cool_down)
                        );

                        incrementRescueTriggerCount("Robot Stuck");

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

            if((ros::Time::now()-goal_start_time).toSec() > params.abort_timeout) {
                ROS_WARN("Goal Total Timeout");
                failed_frontiers.emplace_back(
                    current_goal.target_pose.pose.position.x,
                    current_goal.target_pose.pose.position.y,
                    ros::Time::now(),
                    ros::Time::now() + ros::Duration(params.failed_frontier_cool_down)
                );

                incrementRescueTriggerCount("Goal Total Timeout");

                requestCancel();
                return;
            }

            if ((ros::Time::now() - last_obstacle_check_time).toSec() > params.obstacle_check_interval) {
                last_obstacle_check_time = ros::Time::now();
                if (checkNearbyObstacleRunning()) {
                    if (!obstacle_suspected) {
                        obstacle_suspected = true;
                        first_obstacle_detect_time = ros::Time::now();
                    } else if ((ros::Time::now() - first_obstacle_detect_time).toSec() > params.nearby_obstacle_confirm_delay) {
                        ROS_WARN("Obstacle Confirmed (Nearby High Cost) - Cancelling Goal");
                        requestCancel();
                    }
                } else { obstacle_suspected = false; }
            }
        } else {
            if((ros::Time::now()-goal_start_time).toSec() > params.rescue_plan_timeout * 2.0) {
                ROS_WARN("Rescue Goal Timeout");
                requestCancel();
                return;
            }
        }
    }

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO(is_rescue_mode ? "Rescue Successful!" : "Goal Reached");

        // 删除已到达的点（从 frontier_buffer 中清除）
        double gx = current_goal.target_pose.pose.position.x;
        double gy = current_goal.target_pose.pose.position.y;
        frontier_buffer.erase(
            std::remove_if(frontier_buffer.begin(), frontier_buffer.end(),
                [&](const geometry_msgs::PointStamped& p){
                    return hypot(p.point.x - gx, p.point.y - gy) < params.failed_frontier_tolerance;
                }),
            frontier_buffer.end()
        );

        if (!is_rescue_mode) {
            failed_frontiers.emplace_back(
                current_goal.target_pose.pose.position.x,
                current_goal.target_pose.pose.position.y,
                ros::Time::now(),
                ros::Time::now() + ros::Duration(params.failed_frontier_cool_down * 2.0)
            );
        }
        goal_active = false;
        plan_received = false;
        waiting_for_plan = false;
        distance_initialized = false;
        consecutive_plan_failures = 0;

        rescue_trigger_count = 0;
        ROS_INFO("[Rescue Counter] Goal completed, rescue_trigger_count reset to 0");

        if (is_rescue_mode) {
            ROS_WARN("Rescue goal completed successfully. Exiting rescue mode.");

            double rescue_duration = (ros::Time::now() - rescue_mode_start_time).toSec();
            rescue_mode_total_duration += rescue_duration;
            for (auto& f : failed_frontiers) {
                f.cool_until += ros::Duration(rescue_duration);
            }
            ROS_INFO("[Rescue Exit] Added %.1fs rescue duration to blacklist timers (total: %.1fs).",
                     rescue_duration, rescue_mode_total_duration);

            is_rescue_mode = false;
            rescue_attempt_count = 0;
            rescue_goal_start_time = ros::Time(0);
            rescue_mode_start_time = ros::Time(0);
            last_stuck_abort_check_time = ros::Time(0);
            last_robot_position.x = 0.0;
            last_robot_position.y = 0.0;
            publishRescueInfo();
        }
        return;
    }
}

void FrontierGoalManager::spin()
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
            if (!is_rescue_mode && consecutive_plan_failures >= params.rescue_trigger_failures) {
                ROS_WARN_STREAM("Consecutive plan failures reached " << consecutive_plan_failures
                    << ". Activating ULTIMATE RESCUE MODE.");

                consecutive_plan_failures = 0;
                rescue_trigger_count = 0;
                just_canceled = false;

                is_rescue_mode = true;
                rescue_attempt_count = 0;
                rescue_mode_start_time = ros::Time::now();
                double sum_x = 0, sum_y = 0;
                for (const auto& p : frontier_buffer) {
                    sum_x += p.point.x;
                    sum_y += p.point.y;
                }
                rescue_center_x = sum_x / frontier_buffer.size();
                rescue_center_y = sum_y / frontier_buffer.size();
                ROS_WARN_STREAM(">>> RESCUE MODE ACTIVATED <<< Center: (" << rescue_center_x << ", " << rescue_center_y << ")");
                publishRescueInfo();
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

            if (is_rescue_mode) {
                if (rescue_attempt_count >= params.rescue_max_attempts) {
                    ROS_ERROR("Rescue Max Attempts Reached (%d). Exiting Rescue Mode.", params.rescue_max_attempts);

                    double rescue_duration = (ros::Time::now() - rescue_mode_start_time).toSec();
                    rescue_mode_total_duration += rescue_duration;
                    for (auto& f : failed_frontiers) {
                        f.cool_until += ros::Duration(rescue_duration);
                    }
                    ROS_INFO("[Rescue Exit] Added %.1fs rescue duration to blacklist timers (total: %.1fs).",
                             rescue_duration, rescue_mode_total_duration);

                    is_rescue_mode = false;
                    rescue_attempt_count = 0;
                    rescue_goal_start_time = ros::Time(0);
                    rescue_mode_start_time = ros::Time(0);
                    last_stuck_abort_check_time = ros::Time(0);
                    last_robot_position.x = 0.0;
                    last_robot_position.y = 0.0;
                    rescue_trigger_count = 0;
                    consecutive_plan_failures = 0;
                    ROS_INFO("[Rescue Exit] Max attempts reached, counters reset.");
                    publishRescueInfo();
                } else if (!attemptRescueGoal()) {
                    ROS_WARN_THROTTLE(2.0, "Rescue: Failed to generate point (Attempt %d/%d). Retrying...",
                                      rescue_attempt_count, params.rescue_max_attempts);
                }
            }
        }

        if (!is_rescue_mode && rescue_trigger_count >= params.rescue_trigger_count_threshold) {
            ROS_WARN_STREAM("Rescue trigger count reached " << rescue_trigger_count
                << " (ABORTED/Stuck/Timeout). Activating RESCUE MODE (dual-trigger).");

            rescue_trigger_count = 0;
            consecutive_plan_failures = 0;

            is_rescue_mode = true;
            rescue_attempt_count = 0;
            rescue_mode_start_time = ros::Time::now();

            double sum_x = 0, sum_y = 0;
            for (const auto& p : frontier_buffer) {
                sum_x += p.point.x;
                sum_y += p.point.y;
            }
            rescue_center_x = sum_x / frontier_buffer.size();
            rescue_center_y = sum_y / frontier_buffer.size();
            ROS_WARN_STREAM(">>> RESCUE MODE ACTIVATED <<< Center: ("
                << rescue_center_x << ", " << rescue_center_y << ")");
            publishRescueInfo();
        }

        ros::spinOnce();
        r.sleep();
    }
}
