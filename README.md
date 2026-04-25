# RoboCup2026 自主建图工作区

rera 的 RoboCup2026 自主建图赛项三维建图工作区。

当前默认车模已从 `src/car` 切换为 `src/rebo24`。建议在自己的分支上提交和保存版本，避免直接覆盖公共工作分支。

## 当前链路

- `fast_lio` 负责定位
- `tare_planner` 负责导航探索
- 三维点云转换为二维点云后使用 `hector_slam` 绘制地图，并导出 `GeoTIFF`

## RoboCup 探索链路

当前 RoboCup 迷宫场景使用 `tare_planner` 的专用入口 `explore_robocup.launch`。该入口会将 `scenario` 固定为 `robocup`，并在启动时自动加载 `src/tare_planner/config/robocup.yaml`。

推荐按下面的链路启动各模块：

- `roslaunch rebo24 simbase.launch`：启动仿真底盘与基础 TF
- `roslaunch fast_lio mapping_velodyne.launch`：提供 `/Odometry`
- `roslaunch loam_interface loam_interface.launch`：将 FAST-LIO 输出整理为 `/registered_scan`
- `roslaunch sensor_scan_generation sensor_scan_generation.launch`：生成 `/state_estimation_at_scan`
- `roslaunch terrain_analysis terrain_analysis.launch`：生成 `/terrain_map`
- `roslaunch terrain_analysis_ext terrain_analysis_ext.launch`：生成 `/terrain_map_ext`
- `roslaunch local_planner local_planner.launch`：订阅 `/way_point` 并输出局部路径与控制
- `roslaunch tare_planner explore_robocup.launch`：启动 RoboCup 场景下的 TARE 探索

其中当前链路里的关键话题关系如下：

- `loam_interface` 发布 `/registered_scan`
- `sensor_scan_generation` 发布 `/state_estimation_at_scan`
- `terrain_analysis` 发布 `/terrain_map`
- `terrain_analysis_ext` 发布 `/terrain_map_ext`
- `tare_planner` 发布 `/way_point`
- `local_planner` 订阅 `/way_point`、`/registered_scan`、`/terrain_map` 和 `/navigation_boundary`

## RoboCup 参数说明

`src/tare_planner/config/robocup.yaml` 当前面向“小尺度室内迷宫、窄通道、连续转角、入口较窄”的 RoboCup 地图做了单独收敛，重点不是让车更激进，而是减少入口附近过早判定“探索完成”和路口左右犹豫。

当前这份参数文件的主要方向是：

- 放宽 frontier 保留条件，避免窄入口附近的小 frontier 被过早滤掉
- 加密 `viewpoint_manager` 采样，并减小视点碰撞边界，尽量保留入口内外候选视点
- 启用 `kUseMomentum`，降低方向切换阈值，减少双入口前来回试探
- 上调 `kExtendWayPointDistanceBig` 与 `kLookAheadDistance`，增强已经选中分支后的延续性
- 下调 `kMinAddPointNumSmall` 与 `kMinAddPointNumBig`，避免窄口前因“新增信息不足”直接结束探索

如果需要录包或边界约束，可以直接使用：

```bash
roslaunch tare_planner explore_robocup.launch \
  rosbag_record:=true \
  bag_path:=Desktop \
  bag_name_prefix:=tare_robocup \
  use_boundary:=true
```

调参时建议优先观察：

- `filtered_frontier_cloud`
- candidate viewpoints
- exploring cell marker
- `/way_point` 是否已经稳定落入目标分支深处

## rebo24 包说明

`src/rebo24` 是当前使用的机器人模型包，包含以下内容：

- `urdf/rebo24.urdf`：四轮底盘、LiDAR、相机、IMU 以及 Gazebo 传感器配置
- `launch/gazebo.launch`：启动 Gazebo 空场景、静态 TF 并生成 `rebo24` 模型
- `launch/display.launch`：发布 `robot_description`，启动 `joint_state_publisher_gui`、`robot_state_publisher`，并支持延时启动
- `launch/simbase.launch`：组合 `gazebo.launch` 与 `display.launch`
- `launch/keyboard.launch`：提供 `teleop_twist_keyboard` 键盘控制入口
- `config/joint_names_rebo24.yaml`：关节名称配置

## 依赖

- ROS 1 + `catkin`
- `gazebo_ros`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `teleop_twist_keyboard`
- `tf`
- `rostopic`
- `std_msgs`

## 常用启动命令

```bash
cd /home/lzk/robotcup2026
catkin build rebo24
source devel/setup.bash

# Gazebo + robot_description + joint_state_publisher_gui
roslaunch rebo24 simbase.launch

# 单独启键盘控制
roslaunch rebo24 keyboard.launch

# 启动 FAST-LIO（Velodyne 配置）
roslaunch fast_lio mapping_velodyne.launch

# 仅加载模型显示
roslaunch rebo24 display.launch model:=$(rospack find rebo24)/urdf/rebo24.urdf
```

## 当前仿真约定

- `gazebo.launch` 当前默认包含 `gazebo_ros/launch/empty_world.launch`
- `rebo24` 当前静态 TF 树约定为 `body -> base_footprint -> base_link`
- `body -> base_footprint` 当前为零位姿，`base_footprint -> base_link` 当前为 `z=0.025`
- 将 `body` 作为最上层节点，是为了避免 `base_footprint` 同时挂在多个父节点下而触发 `TF_REPEATED_DATA` 警告
- 差速插件参数已按 `rebo24` 当前模型几何对齐：`wheelSeparation=0.466000391758278`、`wheelDiameter=0.050`
- LiDAR 通过 `libgazebo_ros_velodyne_laser.so` 发布 `/velodyne_points`
- 相机链包含 depth、infra1、infra2、color 与 IMU 相关坐标系
- Gazebo IMU 插件当前仍挂在 `camera_imu_frame`，发布 `/camera/imu`；`imu_link` 目前没有单独的 Gazebo IMU 插件
- `src/fast_lio/config/velodyne.yaml` 当前按 `lidar -> camera_imu_frame` 对齐，`extrinsic_T=[-0.08376, 0.005038918391, 0.051879206396]`
- `display.launch` 当前会尝试加载 `$(find rebo24)/urdf.rviz`；如果本地没有该文件，需要先补配置或注释 RViz 节点
