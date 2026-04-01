#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class GlobalCostmapTester
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    
    ros::Subscriber costmap_sub;
    ros::Publisher visualize_pub;
    
    nav_msgs::OccupancyGrid latest_costmap;
    bool has_costmap;
    int callback_count;
    
public:
    GlobalCostmapTester() : pnh("~"), has_costmap(false), callback_count(0)
    {
        std::string input_topic = pnh.param<std::string>("input_topic", "/move_base/global_costmap/costmap");
        std::string output_topic = pnh.param<std::string>("output_topic", "/global_costmap_tester/visualize");
        
        ROS_INFO("============================================================");
        ROS_INFO("[GlobalCostmapTester] Subscribing to: %s", input_topic.c_str());
        ROS_INFO("[GlobalCostmapTester] Publishing to: %s", output_topic.c_str());
        ROS_INFO("[GlobalCostmapTester] Queue size: 10 (same as rrt_goal_node)");
        ROS_INFO("============================================================");
        
        // 【关键修改】使用 waitForMessage 获取第一个完整地图（latched）
        ROS_INFO("[GlobalCostmapTester] Waiting for latched costmap (up to 10s)...");
        nav_msgs::OccupancyGrid::ConstPtr initial_msg = 
            ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(input_topic, nh, ros::Duration(10.0));
        
        if (initial_msg != nullptr)
        {
            latest_costmap = *initial_msg;  // 深拷贝保存
            has_costmap = true;
            int w = latest_costmap.info.width;
            int h = latest_costmap.info.height;
            ROS_INFO("[GlobalCostmapTester] Got initial costmap: %dx%d, resolution=%.3f", w, h, latest_costmap.info.resolution);
        }
        else
        {
            ROS_ERROR("[GlobalCostmapTester] Failed to get initial costmap!");
        }
        
        // 订阅 - 接收后续更新
        costmap_sub = nh.subscribe(
            input_topic, 
            10,  // 队列深度与 rrt_goal_node 一致
            &GlobalCostmapTester::costmapCallback, 
            this
        );
        
        // 发布 - 使用 latch=true，确保 rviz 能收到完整地图
        visualize_pub = nh.advertise<nav_msgs::OccupancyGrid>(
            output_topic, 
            10,
            true  // 使用 latch
        );
    }
    
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        // 与 rrt_goal_node 的 globalCostmapCallback 完全一致
        latest_costmap = *msg;  // 深拷贝
        has_costmap = true;
        callback_count++;
        
        // 前 3 次和每 100 次输出详细信息
        if (callback_count <= 3 || callback_count % 100 == 0)
        {
            printCostmapInfo(msg, callback_count);
        }
    }
    
    void printCostmapInfo(const nav_msgs::OccupancyGrid::ConstPtr& msg, int count)
    {
        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;
        double stamp = msg->header.stamp.toSec();
        double age = (ros::Time::now() - msg->header.stamp).toSec();
        
        // 计算物理尺寸
        double physical_width = width * resolution;
        double physical_height = height * resolution;
        
        // 统计代价值分布
        int unknown_count = 0;
        int free_count = 0;
        int occupied_count = 0;
        int high_cost_count = 0;
        int low_cost_count = 0;
        
        for (size_t i = 0; i < msg->data.size(); ++i)
        {
            signed char value = msg->data[i];
            if (value == -1) unknown_count++;
            else if (value == 0) free_count++;
            else if (value >= 100) occupied_count++;
            else if (value >= 50) high_cost_count++;
            else low_cost_count++;
        }
        
        int total = msg->data.size();
        double unknown_pct = total > 0 ? 100.0 * unknown_count / total : 0;
        double free_pct = total > 0 ? 100.0 * free_count / total : 0;
        double occupied_pct = total > 0 ? 100.0 * occupied_count / total : 0;
        double high_cost_pct = total > 0 ? 100.0 * high_cost_count / total : 0;
        double low_cost_pct = total > 0 ? 100.0 * low_cost_count / total : 0;
        
        ROS_INFO("============================================================");
        ROS_INFO("[GlobalCostmapTester] Message #%d", count);
        ROS_INFO("------------------------------------------------------------");
        ROS_INFO("Basic Info:");
        ROS_INFO("  - Stamp: %.3f (age: %.3fs)", stamp, age);
        ROS_INFO("  - Size: %d x %d cells", width, height);
        ROS_INFO("  - Resolution: %.3f m/cell", resolution);
        ROS_INFO("  - Physical Size: %.2f x %.2f meters", physical_width, physical_height);
        ROS_INFO("  - Total Cells: %d", total);
        ROS_INFO(" ");
        ROS_INFO("Cost Distribution:");
        ROS_INFO("  - Unknown (-1):  %6d (%5.1f%%)", unknown_count, unknown_pct);
        ROS_INFO("  - Free (0):      %6d (%5.1f%%)", free_count, free_pct);
        ROS_INFO("  - Low (1-49):    %6d (%5.1f%%)", low_cost_count, low_cost_pct);
        ROS_INFO("  - High (50-99):  %6d (%5.1f%%)", high_cost_count, high_cost_pct);
        ROS_INFO("  - Occupied (100):%6d (%5.1f%%)", occupied_count, occupied_pct);
        ROS_INFO("============================================================");
    }
    
    void spin()
    {
        ros::Rate rate(10.0);  // 10 Hz
        
        ROS_INFO("[GlobalCostmapTester] Starting publish loop at 10 Hz...");
        ROS_INFO("[GlobalCostmapTester] Add '/global_costmap_tester/visualize' to RViz to visualize");
        
        while (ros::ok())
        {
            ros::spinOnce();  // 必须调用，否则回调不执行
            
            if (has_costmap)
            {
                visualize_pub.publish(latest_costmap);
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_costmap_tester");
    GlobalCostmapTester tester;
    
    // 与 rrt_goal_node 的 main 函数一致
    tester.spin();
    return 0;
}
