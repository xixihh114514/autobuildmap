cmake_minimum_required(VERSION 3.0.2)
project(image_map_gazebo)
find_package(catkin REQUIRED COMPONENTS rospy gazebo_ros)
catkin_package()
install(DIRECTORY launch worlds scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
