cmake_minimum_required(VERSION 3.0.2)
project(visual_obstacle_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgproc objdetect)

if(NOT OpenVINO_DIR)
  execute_process(
    COMMAND
      ${Python3_EXECUTABLE}
      -c
      "import importlib.util, pathlib, sys; spec = importlib.util.find_spec('openvino'); pkg = pathlib.Path(spec.origin).resolve().parent if spec and spec.origin else None; candidates = [pkg / 'runtime' / 'cmake', pkg / 'cmake'] if pkg else []; existing = [str(path) for path in candidates if path.exists()]; print(existing[0] if existing else '')"
    OUTPUT_VARIABLE OPENVINO_DIR_FROM_PYTHON
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(OPENVINO_DIR_FROM_PYTHON)
    set(OpenVINO_DIR "${OPENVINO_DIR_FROM_PYTHON}" CACHE PATH "OpenVINO CMake config path")
  endif()
endif()

if(NOT OpenVINO_DIR)
  message(FATAL_ERROR "OpenVINO C++ runtime was not found. Install openvino-dev or set OpenVINO_DIR manually.")
endif()

find_package(OpenVINO REQUIRED)

if(DEFINED OpenVINO_GLIBCXX_USE_CXX11_ABI AND NOT OpenVINO_GLIBCXX_USE_CXX11_ABI STREQUAL "")
  remove_definitions(-D_GLIBCXX_USE_CXX11_ABI=${OpenVINO_GLIBCXX_USE_CXX11_ABI})
endif()

execute_process(
  COMMAND
    ${Python3_EXECUTABLE}
    -c
    "import importlib.util, pathlib; spec = importlib.util.find_spec('openvino'); pkg = pathlib.Path(spec.origin).resolve().parent if spec and spec.origin else None; candidates = [pkg / 'libs', pkg / 'runtime' / 'lib' / 'intel64'] if pkg else []; existing = [str(path) for path in candidates if path.exists()]; print(existing[0] if existing else '')"
  OUTPUT_VARIABLE OPENVINO_RUNTIME_LIB_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    message_filters
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/visual_obstacle_detector.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_library(person_openvino_backend SHARED
  src/openvino_person_backend.cpp
)
add_executable(person_global_localizer_node
  src/person_global_localizer_node.cpp
)
add_executable(qr_global_localizer_node
  src/qr_global_localizer_node.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(person_global_localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(qr_global_localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(person_openvino_backend
  PRIVATE
    openvino::runtime
)
target_link_libraries(person_global_localizer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  person_openvino_backend
)
target_link_libraries(qr_global_localizer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if(OPENVINO_RUNTIME_LIB_DIR)
  set_target_properties(person_openvino_backend PROPERTIES
    BUILD_RPATH "${OPENVINO_RUNTIME_LIB_DIR}"
    INSTALL_RPATH "${OPENVINO_RUNTIME_LIB_DIR}"
  )
  set_target_properties(person_global_localizer_node PROPERTIES
    BUILD_RPATH "${OPENVINO_RUNTIME_LIB_DIR}"
    INSTALL_RPATH "${OPENVINO_RUNTIME_LIB_DIR}"
  )
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/person_global_localizer.py
  scripts/qr_global_localizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  person_openvino_backend
  person_global_localizer_node
  qr_global_localizer_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_visual_obstacle_detector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
