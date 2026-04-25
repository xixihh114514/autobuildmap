# RoboCup2026 自主建图工作区

rera 的 RoboCup2026 自主建图赛项三维建图工作区。

当前默认车模已从 `src/car` 切换为 `src/rebo24`。建议在自己的分支上提交和保存版本，避免直接覆盖公共工作分支。

## 当前链路

- `fast_lio` 负责定位
- `tare_planner` 负责导航探索
- 三维点云转换为二维点云后使用 `hector_slam` 绘制地图，并导出 `GeoTIFF`

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
