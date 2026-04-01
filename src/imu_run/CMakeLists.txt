cmake_minimum_required(VERSION 3.0.2)
project(imu_run)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_motor_control
  src/imu_motor_control.cpp
)

target_link_libraries(imu_motor_control
  ${catkin_LIBRARIES}
)

install(TARGETS imu_motor_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/gimbal_keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
