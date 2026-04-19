# Install script for directory: /home/rera/robocup2026/src/visual_calibration

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rera/robocup2026/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rera/robocup2026/build/visual_calibration/catkin_generated/installspace/visual_calibration.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/visual_calibration/cmake" TYPE FILE FILES
    "/home/rera/robocup2026/build/visual_calibration/catkin_generated/installspace/visual_calibrationConfig.cmake"
    "/home/rera/robocup2026/build/visual_calibration/catkin_generated/installspace/visual_calibrationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/visual_calibration" TYPE FILE FILES "/home/rera/robocup2026/src/visual_calibration/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/visual_calibration/isual_callibration" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/visual_calibration/isual_callibration")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/visual_calibration/isual_callibration"
         RPATH "/home/rera/robocup2026/third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime/lib/intel64:/home/rera/robocup2026/third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime/3rdparty/tbb/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/visual_calibration" TYPE EXECUTABLE FILES "/home/rera/robocup2026/devel/lib/visual_calibration/isual_callibration")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/visual_calibration/isual_callibration" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/visual_calibration/isual_callibration")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/visual_calibration/isual_callibration"
         OLD_RPATH "/home/rera/robocup2026/third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime/lib/intel64:/home/rera/robocup2026/third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime/3rdparty/tbb/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "/home/rera/robocup2026/third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime/lib/intel64:/home/rera/robocup2026/third_party/l_openvino_toolkit_ubuntu20_2024.6.0.17404.4c0f47d2335_x86_64/runtime/3rdparty/tbb/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/visual_calibration/isual_callibration")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/visual_calibration" TYPE DIRECTORY FILES "/home/rera/robocup2026/src/visual_calibration/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/visual_calibration" TYPE DIRECTORY FILES "/home/rera/robocup2026/src/visual_calibration/models")
endif()

