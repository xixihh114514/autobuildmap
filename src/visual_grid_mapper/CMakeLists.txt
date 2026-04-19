cmake_minimum_required(VERSION 3.0.2)
project(visual_grid_mapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hector_geotiff
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    hector_geotiff
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(visual_grid_mapper_node src/visual_grid_mapper_node.cpp)
target_link_libraries(visual_grid_mapper_node
  ${catkin_LIBRARIES}
)

add_library(visual_grid_mapper_geotiff_plugins src/visual_geotiff_plugin.cpp)
add_dependencies(visual_grid_mapper_geotiff_plugins ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_grid_mapper_geotiff_plugins
  ${catkin_LIBRARIES}
)

install(TARGETS visual_grid_mapper_node visual_grid_mapper_geotiff_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES visual_grid_mapper_geotiff_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
