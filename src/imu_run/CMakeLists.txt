cmake_minimum_required(VERSION 3.0.2)
project(imu_run)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_closed_loop_control src/imu_closed_loop_control.cpp)
target_link_libraries(imu_closed_loop_control ${catkin_LIBRARIES} pthread)

add_executable(imu_position_speed_control src/imu_position_speed_control.cpp)
target_link_libraries(imu_position_speed_control ${catkin_LIBRARIES} pthread)

install(TARGETS imu_closed_loop_control imu_position_speed_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/gimbal_keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
