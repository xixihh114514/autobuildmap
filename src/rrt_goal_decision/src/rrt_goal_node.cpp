#include <ros/ros.h>
#include <rrt_goal_decision/frontier_goal_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_goal_decision");
    FrontierGoalManager manager;
    manager.spin();
    return 0;
}
