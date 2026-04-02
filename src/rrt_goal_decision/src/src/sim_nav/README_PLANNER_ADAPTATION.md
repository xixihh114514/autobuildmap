# 局部规划器适配完成 - TEB 与 DWA 平级部署

## ✅ 适配完成

现在 `dwa_local_planner` 已经从 `navigation` 目录移出，与 `teb_local_planner` 平级：

```
src/
├── teb_local_planner/         # TEB 局部规划器（你的修改版）
├── dwa_local_planner/         # DWA 局部规划器（ROS 官方）
├── sim_nav/                   # 导航配置包
│   ├── config/
│   │   ├── local_planner.yaml      # TEB 参数
│   │   └── dwa_local_planner.yaml  # DWA 参数（新增）
│   ├── launch/
│   │   └── nav.launch              # 支持切换规划器
│   └── scripts/
│       └── switch_planner.sh       # 切换脚本（新增）
```

## 🚀 快速开始

### 方法 1: 使用切换脚本

```bash
cd ~/robotcup2026/src/sim_nav

# 查看当前配置
./scripts/switch_planner.sh

# 切换到 TEB（适合建图、狭窄空间）
./scripts/switch_planner.sh teb

# 切换到 DWA（适合快速导航）
./scripts/switch_planner.sh dwa
```

### 方法 2: 直接修改 launch 文件

编辑 `src/sim_nav/launch/nav.launch`:

```xml
<!-- 使用 TEB -->
<arg name="local_planner" default="teb_local_planner/TebLocalPlannerROS" />

<!-- 使用 DWA -->
<arg name="local_planner" default="dwa_local_planner/DWAPlannerROS" />
```

### 方法 3: 命令行临时切换

```bash
# 使用 TEB 启动
roslaunch sim_nav nav.launch local_planner:=teb_local_planner/TebLocalPlannerROS

# 使用 DWA 启动
roslaunch sim_nav nav.launch local_planner:=dwa_local_planner/DWAPlannerROS
```

## 📊 规划器对比

| 特性 | TEB | DWA | 推荐使用场景 |
|------|-----|-----|-------------|
| **轨迹平滑度** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | TEB: 精密导航 |
| **计算速度** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | DWA: 快速响应 |
| **参数复杂度** | 复杂 (~50 个) | 简单 (~20 个) | 新手：DWA |
| **狭窄空间** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | TEB: 建图探索 |
| **开阔地带** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | DWA: 快速巡航 |
| **履带适应性** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | TEB: 履带机器人 |

## 🔧 推荐配置

### 履带机器人 - 探索建图（推荐 TEB）

```bash
./scripts/switch_planner.sh teb
```

**参数调整** (`src/sim_nav/config/local_planner.yaml`):
```yaml
TebLocalPlannerROS:
  max_vel_x: 0.25              # 保守速度
  max_vel_theta: 1.5           # 减少打滑
  weight_viapoint: 80.0        # 强跟随路径
  weight_kinematics_nh: 2000   # 履带运动学约束
```

### 履带机器人 - 快速导航（使用 DWA）

```bash
./scripts/switch_planner.sh dwa
```

**参数调整** (`src/sim_nav/config/dwa_local_planner.yaml`):
```yaml
DWAPlannerROS:
  max_vel_x: 0.25              # 履带保守速度
  max_vel_theta: 1.5           # 减少转向打滑
  path_distance_bias: 40.0     # 强跟随路径
  occdist_bias: 0.02           # 保守避障
```

## 📝 编译说明

### 首次编译（包含 DWA 源码）

```bash
cd ~/robotcup2026

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译所有包
catkin_make

# 或只编译规划器相关包（更快）
catkin_make -DCATKIN_WHITELIST_PACKAGES="teb_local_planner;dwa_local_planner;sim_nav"
```

### 验证编译

```bash
# 检查规划器是否可用
rospack find teb_local_planner
rospack find dwa_local_planner

# 应该都显示源码路径（不是/opt/ros）
```

## 🐛 常见问题

### Q1: 切换规划器后机器人不动

**解决**:
```bash
# 重启 move_base
rosnode kill /move_base
roslaunch sim_nav nav.launch
```

### Q2: 参数不生效

**解决**:
```bash
# 清除参数缓存
rosparam clear

# 重新启动
roslaunch sim_nav nav.launch
```

### Q3: 编译错误 - 找不到规划器

**解决**:
```bash
cd ~/robotcup2026
catkin_make clean
catkin_make -DCATKIN_WHITELIST_PACKAGES="teb_local_planner;dwa_local_planner"
source devel/setup.bash
```

## 📚 相关文档

- `PLANNER_SWITCHING_GUIDE.md` - 详细的规划器切换指南
- `BUILD_DWA_FROM_SOURCE.md` - DWA 源码编译说明
- `config/local_planner.yaml` - TEB 参数详解
- `config/dwa_local_planner.yaml` - DWA 参数详解

## 🎯 下一步

1. **测试 TEB**:
   ```bash
   ./scripts/switch_planner.sh teb
   roslaunch sim_nav nav.launch
   ```

2. **测试 DWA**:
   ```bash
   ./scripts/switch_planner.sh dwa
   roslaunch sim_nav nav.launch
   ```

3. **根据测试结果调整参数**

## ✅ 完成清单

- [x] 移动 `dwa_local_planner` 到 `src/` 目录
- [x] 修改 `nav.launch` 支持规划器切换
- [x] 创建 `dwa_local_planner.yaml` 配置文件
- [x] 创建 `switch_planner.sh` 切换脚本
- [x] 创建完整文档
- [x] 添加中文注释
- [x] 履带机器人参数优化

现在你可以根据任务需求灵活切换规划器了！🎉
