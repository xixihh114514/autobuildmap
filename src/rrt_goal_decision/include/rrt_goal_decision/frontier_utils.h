#ifndef RRT_GOAL_DECISION_FRONTIER_UTILS_H
#define RRT_GOAL_DECISION_FRONTIER_UTILS_H

#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

namespace frontier_utils {

// 地图坐标转换结果
struct MapCoords {
    int mx;
    int my;
    bool valid;
    MapCoords() : mx(0), my(0), valid(false) {}
    MapCoords(int _mx, int _my) : mx(_mx), my(_my), valid(true) {}
};

// 世界坐标转地图网格坐标（四舍五入对齐）
MapCoords worldToMapCoords(double x, double y, const nav_msgs::OccupancyGrid& map);

// 检查位移并更新参考位置
bool checkDisplacement(double new_x, double new_y, double &ref_x, double &ref_y, double threshold = 0.5);

} // namespace frontier_utils

#endif // RRT_GOAL_DECISION_FRONTIER_UTILS_H
