#include "rrt_goal_decision/frontier_utils.h"

namespace frontier_utils {

MapCoords worldToMapCoords(double x, double y, const nav_msgs::OccupancyGrid& map)
{
    double res = map.info.resolution;
    if (res <= 1e-6) return MapCoords();
    int mx = static_cast<int>(std::round((x - map.info.origin.position.x) / res));
    int my = static_cast<int>(std::round((y - map.info.origin.position.y) / res));
    if (mx < 0 || mx >= static_cast<int>(map.info.width) || my < 0 || my >= static_cast<int>(map.info.height)) {
        return MapCoords();
    }
    return MapCoords(mx, my);
}

bool checkDisplacement(double new_x, double new_y, double &ref_x, double &ref_y, double threshold)
{
    double d = std::hypot(new_x - ref_x, new_y - ref_y);
    ref_x = new_x;
    ref_y = new_y;
    return d > threshold;
}

} // namespace frontier_utils
