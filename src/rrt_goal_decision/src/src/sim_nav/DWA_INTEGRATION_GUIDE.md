# DWA 局部规划器集成说明

## 快速开始

### 1. 安装 DWA 功能包

```bash
# ROS Noetic
sudo apt-get install ros-noetic-dwa-local-planner

# 验证安装
rospack find dwa_local_planner
```

### 2. 切换局部规划器

#### 方法 A: 修改 launch 文件（推荐）

编辑 `src/sim_nav/sim_nav/launch/nav.launch`:

```xml
<!-- 使用 DWA -->
<arg name="local_planner" default="dwa_local_planner/DWAPlannerROS" />

<!-- 使用 TEB -->
<arg name="local_planner" default="teb_local_planner/TebLocalPlannerROS" />
```

#### 方法 B: 命令行覆盖

```bash
# 使用 DWA 启动
roslaunch sim_nav nav.launch local_planner:=dwa_local_planner/DWAPlannerROS

# 使用 TEB 启动
roslaunch sim_nav nav.launch local_planner:=teb_local_planner/TebLocalPlannerROS
```

### 3. 验证是否生效

```bash
# 查看当前使用的局部规划器
rosparam get /move_base/base_local_planner

# 应该输出：dwa_local_planner/DWAPlannerROS
```

## 参数配置

### 核心参数说明

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|----------|
| `max_vel_x` | 0.3 | 最大前进速度 | 履带：0.2~0.4 |
| `max_vel_theta` | 1.5 | 最大角速度 | 履带：1.0~2.0 |
| `acc_lim_x` | 0.5 | 线加速度 | 履带：0.3~0.8 |
| `acc_lim_theta` | 2.0 | 角加速度 | 履带：1.5~3.0 |
| `path_distance_bias` | 32.0 | 路径跟随权重 | 越大越贴近全局路径 |
| `goal_distance_bias` | 24.0 | 目标吸引权重 | 越大越激进 |
| `occdist_bias` | 0.01 | 避障权重 | 越大越保守 |

### 履带机器人推荐配置

```yaml
DWAPlannerROS:
  # 运动学参数（保守）
  max_vel_x: 0.25
  max_vel_theta: 1.5
  acc_lim_x: 0.3
  acc_lim_theta: 1.5
  
  # 采样参数
  vx_samples: 20
  vtheta_samples: 40
  sim_time: 1.5
  
  # 评价函数（平衡型）
  path_distance_bias: 40.0    # 强跟随路径
  goal_distance_bias: 20.0    # 适度激进
  occdist_bias: 0.02          # 保守避障
```

### 差速轮机器人推荐配置

```yaml
DWAPlannerROS:
  # 运动学参数（激进）
  max_vel_x: 0.5
  max_vel_theta: 3.0
  acc_lim_x: 1.0
  acc_lim_theta: 3.0
  
  # 采样参数
  vx_samples: 30
  vtheta_samples: 50
  sim_time: 1.0
  
  # 评价函数（快速型）
  path_distance_bias: 25.0
  goal_distance_bias: 30.0
  occdist_bias: 0.01
```

## 调试技巧

### 1. 可视化轨迹

```bash
# 启动 rqt_reconfigure
rosrun rqt_reconfigure rqt_reconfigure

# 启用轨迹发布
# 在 /move_base/DWAPlannerROS 下：
# - publish_traj_pc = true
# - publish_cost_grid_pc = true

# 在 RViz 中添加：
# - By topic -> /move_base/DWAPlannerROS/cost_cloud (PointCloud2)
# - By topic -> /move_base/DWAPlannerROS/trajectory_cloud (PointCloud2)
```

### 2. 动态调参

```bash
# 实时调整路径跟随权重
rosparam set /move_base/DWAPlannerROS/path_distance_bias 50.0

# 实时调整最大速度
rosparam set /move_base/DWAPlannerROS/max_vel_x 0.4
```

### 3. 查看日志

```bash
# 查看 DWA 详细日志
rosconsole set /move_base/DWAPlannerROS info

# 或查看 ROS 日志
tail -f ~/.ros/log/latest/move_base*.log
```

## 常见问题

### Q1: 机器人原地打转
**原因**: `max_vel_theta` 太大或 `acc_lim_theta` 太小
**解决**: 
```yaml
max_vel_theta: 1.5
acc_lim_theta: 2.0
```

### Q2: 机器人不避障
**原因**: `occdist_bias` 太小或 `costmap` 膨胀半径太小
**解决**:
```yaml
occdist_bias: 0.05
# 同时检查 costmap 的 inflation_radius
```

### Q3: 机器人走S形路线
**原因**: `path_distance_bias` 太小
**解决**:
```yaml
path_distance_bias: 40.0  # 增大到 30~50
```

### Q4: 机器人到不了目标
**原因**: `xy_goal_tolerance` 太小或 `latch_xy_goal_tolerance` 问题
**解决**:
```yaml
xy_goal_tolerance: 0.2
latch_xy_goal_tolerance: false
```

### Q5: 编译错误
```
Cannot find dwa_local_planner
```
**解决**:
```bash
sudo apt-get install ros-noetic-dwa-local-planner
source ~/catkin_ws/devel/setup.bash
```

## 与 TEB 的对比

| 特性 | DWA | TEB |
|------|-----|-----|
| **计算速度** | 快 | 较慢 |
| **轨迹平滑度** | 一般 | 优秀 |
| **参数数量** | 少（~20 个） | 多（~50 个） |
| **调参难度** | 简单 | 复杂 |
| **适用场景** | 简单环境、高速导航 | 复杂环境、狭窄空间 |
| **履带适应性** | 好（通过参数调整） | 优秀（原生支持） |

## 性能优化

### 提高计算速度
```yaml
vx_samples: 15          # 减少采样数
vtheta_samples: 30
sim_time: 1.0           # 缩短仿真时间
controller_frequency: 10.0  # 降低控制频率
```

### 提高轨迹质量
```yaml
vx_samples: 30          # 增加采样数
vtheta_samples: 50
sim_time: 2.0           # 延长仿真时间
sim_granularity: 0.02   # 细化仿真粒度
```

## 文件结构

```
sim_nav/config/
├── dwa_local_planner.yaml      # DWA 参数配置（新增）
├── local_planner.yaml          # TEB 参数配置（原有）
├── costmap_common_params.yaml  # 代价地图公共参数
├── global_costmap_params.yaml  # 全局代价地图
├── local_costmap_params.yaml   # 局部代价地图
└── global_planner.yaml         # 全局规划器

sim_nav/launch/
└── nav.launch                  # 导航启动文件（已修改）
```

## 参考资料

- [ROS Wiki - dwa_local_planner](http://wiki.ros.org/dwa_local_planner)
- [DWA 算法原论文](https://www.ri.cmu.edu/pub_files/pub3/fox_dieter_1997_6/fox_dieter_1997_6.pdf)
- [ROS Navigation Tuning Guide](https://roboticsbackend.com/ros-navigation-tuning-guide/)
