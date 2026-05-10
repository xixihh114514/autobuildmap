# RERA RoboCup China 2026 Autonomous Mapping Repository

本仓库属于 `RERA` 团队 RoboCup 中国机器人大赛环境自主建图赛项项目仓库，用于三维与二维自主建图方案的开发、仿真验证与实车部署。

仓库当前围绕两条主技术路线维护：

- 三维方案：面向三维点云感知、自主导航、探索建图与目标标记
- 二维方案：面向二维激光 SLAM、导航探索与目标标记

## Repository Overview

本仓库主要包含以下能力模块：

- 三维定位、建图与导航相关功能包
- 二维 SLAM、导航与探索相关功能包
- Gazebo 仿真环境与机器人模型
- 视觉识别与图像标记相关功能
- 地图生成与导出相关工具链

## Branch Description

仓库主要分支说明如下：

- `master`
  - 主要用于三维仿真开发
  - 也是当前主要功能迭代与修改分支
- `real_car_3D`
  - 主要用于三维方案在真实车辆上的部署与适配
- `2dlidar` 方案分支
  - 用于二维方案的仿真与实车共用开发
  - 当前远程分支名为 `robocup26_2dlidar`

建议在个人分支上进行功能开发与调试，再合并到对应公共分支，避免直接覆盖团队工作内容。

## 3D Solution

三维方案主要面向三维自主建图任务，核心技术链路如下：

- 使用 `FAST-LIO` 进行三维激光定位
- 使用 `TARE Planner` 进行导航与自主探索
- 使用 `YOLOv8` 与 `OpenCV` 进行目标识别与标记
- 使用 `hector_slam` 保存 `TIFF` 格式地图

三维方案当前主要对应：

- `master`：三维仿真与主要开发
- `real_car_3D`：三维实车部署

## 2D Solution

二维方案主要面向二维激光自主建图任务，核心技术链路如下：

- 使用 `hector_slam` 进行二维 `SLAM` 并保存 `TIFF` 格式地图
- 使用 `move_base` 进行导航
- 使用 `explore` 与 `RRT` 融合进行自主探索
- 使用 `YOLOv8` 与 `OpenCV` 进行目标识别与标记

二维方案对应当前远程分支：

- `robocup26_2dlidar`

## Main Packages

仓库中已包含的核心功能包包括但不限于：

- `fast_lio`
- `tare_planner`
- `hector_slam`
- `navigation`
- `rebo24`
- `velodyne`
- `rplidar_ros`
- `pointcloud_to_laserscan`
- `realsense-ros`
- `autonomous_exploration_development_environment`

## Workspace Structure

本仓库为标准 `ROS 1 + catkin` 工作区，主要目录如下：

- `src/`：功能包源码
- `build/`：编译输出目录
- `devel/`：开发环境输出目录

## Environment Requirements

建议使用以下基础环境：

- `ROS 1`
- `catkin`
- `Gazebo`
- `OpenCV`
- `YOLOv8` 相关运行环境

不同分支在传感器驱动、实车接口与参数配置上可能存在差异，部署前请根据对应分支进行检查。

## Quick Start

```bash
cd /home/lzk/robotcup2026
catkin build
source devel/setup.bash
```

三维仿真常用启动方式示例：

```bash
# 启动机器人仿真
roslaunch rebo24 simbase.launch

# 启动 FAST-LIO
roslaunch fast_lio mapping_velodyne.launch
```

## Notes

- 当前默认机器人模型位于 `src/rebo24`
- 三维与二维方案使用的传感器、参数配置与启动流程并不完全相同
- 在切换分支后，建议重新检查依赖、编译状态与启动参数

## Maintenance Suggestion

为保证团队协作效率，建议遵循以下约定：

- 将仿真开发与实车部署区分在不同分支维护
- 对重要参数修改保留变更记录
- 优先在独立分支完成测试后再合并到公共分支

## License

如无单独说明，本仓库内各功能包的许可证以其各自源码与上游项目说明为准。
