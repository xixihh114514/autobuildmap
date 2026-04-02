# 局部规划器适配说明

## 目录结构

```
src/
├── teb_local_planner/         # TEB 局部规划器（你的修改版）
├── dwa_local_planner/         # DWA 局部规划器（ROS 官方，从 navigation 移出）
├── sim_nav/
│   ├── config/
│   │   ├── local_planner.yaml      # TEB 参数配置
│   │   └── dwa_local_planner.yaml  # DWA 参数配置
│   ├── launch/
│   │   └── nav.launch              # 导航启动文件（支持切换）
│   └── scripts/
│       └── switch_planner.sh       # 规划器切换脚本
```

## 使用方法

### 方法 1: 使用切换脚本（推荐）

```bash
cd ~/robotcup2026/src/sim_nav

# 查看当前配置
./scripts/switch_planner.sh

# 切换到 TEB
./scripts/switch_planner.sh teb

# 切换到 DWA
./scripts/switch_planner.sh dwa
```

### 方法 2: 修改 launch 文件

编辑 `src/sim_nav/launch/nav.launch`:

```xml
<!-- 使用 TEB -->
<arg name="local_planner" default="teb_local_planner/TebLocalPlannerROS" />

<!-- 使用 DWA -->
<arg name="local_planner" default="dwa_local_planner/DWAPlannerROS" />
```

### 方法 3: 命令行覆盖

```bash
# 临时使用 TEB
roslaunch sim_nav nav.launch local_planner:=teb_local_planner/TebLocalPlannerROS

# 临时使用 DWA
roslaunch sim_nav nav.launch local_planner:=dwa_local_planner/DWAPlannerROS
```

## 验证配置

```bash
# 启动导航
roslaunch sim_nav nav.launch

# 查看当前使用的规划器
rosparam get /move_base/base_local_planner

# 应该输出：
# teb_local_planner/TebLocalPlannerROS
# 或
# dwa_local_planner/DWAPlannerROS
```

## 参数对比

### TEB 配置 (`local_planner.yaml`)

```yaml
TebLocalPlannerROS:
  max_vel_x: 0.25
  max_vel_theta: 2.0
  weight_kinematics_nh: 1000
  weight_viapoint: 50.0
  # ... 更多优化参数
```

**特点**:
- ✅ 轨迹平滑优秀
- ✅ 支持原地转向
- ✅ 适合狭窄空间
- ⚠️ 计算量较大
- ⚠️ 参数复杂（~50 个）

### DWA 配置 (`dwa_local_planner.yaml`)

```yaml
DWAPlannerROS:
  max_vel_x: 0.3
  max_vel_theta: 1.5
  path_distance_bias: 32.0
  goal_distance_bias: 24.0
  # ... 更少参数
```

**特点**:
- ✅ 计算速度快
- ✅ 参数简单（~20 个）
- ✅ 稳定可靠
- ⚠️ 轨迹不如 TEB 平滑
- ⚠️ 狭窄空间表现一般

## 推荐配置

### 履带机器人 - 探索模式（默认 TEB）

```bash
# 使用 TEB（适合建图、狭窄环境）
./scripts/switch_planner.sh teb

# 修改参数（可选）
vim src/sim_nav/config/local_planner.yaml
# 调整：
# - weight_viapoint: 50.0 → 80.0（更强跟随路径）
# - max_vel_theta: 2.0 → 1.5（减少转向打滑）
```

### 履带机器人 - 快速导航（使用 DWA）

```bash
# 使用 DWA（适合已知环境、快速移动）
./scripts/switch_planner.sh dwa

# 修改参数（可选）
vim src/sim_nav/config/dwa_local_planner.yaml
# 调整：
# - path_distance_bias: 32.0 → 40.0（更强跟随路径）
# - max_vel_x: 0.3 → 0.25（履带保守速度）
```

## 性能对比

| 测试场景 | TEB | DWA | 推荐 |
|---------|-----|-----|------|
| **直角弯通过** | 优秀 | 良好 | TEB |
| **狭窄走廊** | 优秀 | 良好 | TEB |
| **开阔地带** | 良好 | 优秀 | DWA |
| **动态避障** | 优秀 | 良好 | TEB |
| **CPU 占用** | ~15% | ~5% | DWA |
| **建图质量** | 优秀 | 良好 | TEB |

## 故障排除

### Q1: 切换后机器人不动

**检查**:
```bash
# 查看规划器是否加载成功
rostopic echo /move_base/TebLocalPlannerROS/local_plan
# 或
rostopic echo /move_base/DWAPlannerROS/local_plan
```

**解决**:
```bash
# 重启 move_base
rosnode kill /move_base
roslaunch sim_nav nav.launch
```

### Q2: 参数不生效

**原因**: 缓存问题

**解决**:
```bash
# 清除 ROS 参数缓存
rosparam clear

# 重新启动
roslaunch sim_nav nav.launch
```

### Q3: 编译错误

```
CMake Error: Cannot find teb_local_planner
```

**解决**:
```bash
cd ~/robotcup2026
catkin_make -DCATKIN_WHITELIST_PACKAGES="teb_local_planner;dwa_local_planner"
source devel/setup.bash
```

## 高级配置

### 混合使用（根据场景切换）

创建场景配置文件 `src/sim_nav/config/planner_scenarios.yaml`:

```yaml
scenarios:
  exploration:  # 探索建图场景
    planner: teb
    params:
      max_vel_x: 0.25
      max_vel_theta: 1.5
      weight_viapoint: 80.0
  
  navigation:   # 已知环境导航
    planner: dwa
    params:
      max_vel_x: 0.4
      max_vel_theta: 2.0
      path_distance_bias: 40.0
  
  rescue:       # 救援模式（狭窄空间）
    planner: teb
    params:
      max_vel_x: 0.15
      max_vel_theta: 1.0
      weight_kinematics_nh: 2000
```

### 自动切换脚本

创建 `auto_switch_planner.sh`:

```bash
#!/bin/bash

# 根据地图大小自动选择规划器
map_size=$(rostopic echo /map | grep "width:" | grep -oP '\d+')

if [ "$map_size" -gt 1000 ]; then
    echo "大地图，使用 DWA（快速导航）"
    ./scripts/switch_planner.sh dwa
else
    echo "小地图，使用 TEB（精确导航）"
    ./scripts/switch_planner.sh teb
fi
```

## 文件清单

### 修改的文件
- ✅ `sim_nav/launch/nav.launch` - 添加规划器选择参数

### 新增的文件
- ✅ `sim_nav/config/dwa_local_planner.yaml` - DWA 参数配置
- ✅ `sim_nav/scripts/switch_planner.sh` - 切换脚本
- ✅ `sim_nav/PLANNER_SWITCHING_GUIDE.md` - 本文档

### 保持不变的文件
- `sim_nav/config/local_planner.yaml` - TEB 参数（原有）
- `src/teb_local_planner/` - TEB 源码（你的修改版）
- `src/dwa_local_planner/` - DWA 源码（从 navigation 移出）

## 参考资料

- [TEB 官方文档](http://wiki.ros.org/teb_local_planner)
- [DWA 官方文档](http://wiki.ros.org/dwa_local_planner)
- [ROS Navigation Tuning Guide](https://roboticsbackend.com/ros-navigation-tuning-guide/)
