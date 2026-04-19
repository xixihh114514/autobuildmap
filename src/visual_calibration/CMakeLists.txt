cmake_minimum_required(VERSION 3.0.2)
project(visual_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  roslib
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

set(OPENVINO_ROOT_HINTS
  $ENV{OPENVINO_ROOT}
  /opt/intel/openvino
  ${PROJECT_SOURCE_DIR}/../../third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime
  ${CMAKE_SOURCE_DIR}/../third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime
)

find_path(OPENVINO_CONFIG_DIR
  NAMES OpenVINOConfig.cmake
  PATHS ${OPENVINO_ROOT_HINTS}
  PATH_SUFFIXES cmake runtime/cmake
)

if(NOT OPENVINO_CONFIG_DIR)
  message(FATAL_ERROR "OpenVINO not found. Set OPENVINO_ROOT or place the runtime archive under third_party/.")
endif()

list(APPEND CMAKE_PREFIX_PATH ${OPENVINO_CONFIG_DIR})
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

get_filename_component(OPENVINO_RUNTIME_ROOT "${OPENVINO_CONFIG_DIR}/.." ABSOLUTE)
set(OPENVINO_TBB_CMAKE_DIR "${OPENVINO_RUNTIME_ROOT}/3rdparty/tbb/lib/cmake/TBB")
set(OPENVINO_TBB_LIB_DIR "${OPENVINO_RUNTIME_ROOT}/3rdparty/tbb/lib")
find_library(OPENVINO_TBB_LIBRARY
  NAMES tbb
  PATHS ${OPENVINO_TBB_LIB_DIR}
  NO_DEFAULT_PATH
)
if(NOT OPENVINO_TBB_LIBRARY)
  message(FATAL_ERROR "OpenVINO TBB library not found under ${OPENVINO_TBB_LIB_DIR}")
endif()

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp roslib sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(isual_callibration src/visual_callibration.cpp)
target_link_libraries(isual_callibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  openvino::runtime
  ${OPENVINO_TBB_LIBRARY}
)
set_target_properties(isual_callibration PROPERTIES
  BUILD_RPATH "${OPENVINO_RUNTIME_ROOT}/lib/intel64;${OPENVINO_TBB_LIB_DIR}"
  INSTALL_RPATH "${OPENVINO_RUNTIME_ROOT}/lib/intel64;${OPENVINO_TBB_LIB_DIR}"
)

install(TARGETS isual_callibration
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
