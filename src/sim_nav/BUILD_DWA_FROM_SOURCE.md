# 从源码编译 DWA Local Planner

## 前提条件

你的工作空间中已经有了 navigation 源码：
```
src/navigation/dwa_local_planner/
```

## 编译步骤

### 1. 确保依赖完整

```bash
cd ~/robotcup2026

# 安装 navigation 栈的系统依赖
rosdep install --from-paths src --ignore-src -r -y

# 或者手动安装关键依赖
sudo apt-get install ros-noetic-base-local-planner \
                     ros-noetic-costmap-2d \
                     ros-noetic-nav-core \
                     ros-noetic-navfn \
                     ros-noetic-global-planner
```

### 2. 编译工作空间

```bash
cd ~/robotcup2026

# 清理旧的编译文件（可选，建议首次编译时执行）
catkin_make clean

# 编译整个工作空间
catkin_make

# 或者只编译 navigation 相关包（更快）
catkin_make -DCATKIN_WHITELIST_PACKAGES="dwa_local_planner;base_local_planner;costmap_2d;nav_core"
```

### 3. 验证编译成功

```bash
# 检查 DWA 插件是否可用
rospack plugins --attrib=plugin nav_core | grep dwa

# 应该输出类似：
# dwa_local_planner  /home/john/robotcup2026/devel/lib/dwa_local_planner/dwa_local_planner.xml
```

### 4. 设置环境变量

```bash
# 确保 sourcing 工作空间
source devel/setup.bash

# 添加到 ~/.bashrc 永久生效
echo "source ~/robotcup2026/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 验证从源码编译

### 方法 1: 检查库文件路径

```bash
# 查找 DWA 库文件
rospack find dwa_local_planner

# 应该显示源码路径，而不是系统路径：
# ✅ /home/john/robotcup2026/src/navigation/dwa_local_planner
# ❌ /opt/ros/noetic/share/dwa_local_planner
```

### 方法 2: 查看编译时间

```bash
# 查看库文件的修改时间
ls -l devel/lib/libdwa_local_planner.*

# 应该是最近编译的时间
```

### 方法 3: 使用 ldd 检查依赖

```bash
# 检查 move_base 的 DWA 依赖
ldd devel/lib/move_base/move_base | grep dwa

# 应该指向 devel/lib 而不是 /opt/ros
```

## 常见问题

### Q1: 编译错误 - 找不到 base_local_planner

**错误信息**:
```
CMake Error at CMakeLists.txt:XX (find_package):
  By not providing "Findbase_local_planner.cmake" in CMAKE_MODULE_PATH this
  project has asked by find_package() to find the package configuration file
  but the project has not found it.
```

**解决**:
```bash
# 确保 base_local_planner 也被编译
catkin_make -DCATKIN_WHITELIST_PACKAGES="base_local_planner;dwa_local_planner"
```

### Q2: 运行时仍然使用系统版本

**原因**: ROS_PACKAGE_PATH 顺序问题

**解决**:
```bash
# 检查 ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH

# 确保工作空间在前面：
# /home/john/robotcup2026/src:/opt/ros/noetic/share

# 如果不是，重新 source
source devel/setup.bash

# 或者修改 ~/.bashrc，确保工作空间在最前面
```

### Q3: 插件加载失败

**错误信息**:
```
Failed to load plugin dwa_local_planner/DWAPlannerROS
```

**解决**:
```bash
# 检查插件描述文件
cat devel/lib/dwa_local_planner/dwa_local_planner.xml

# 应该包含类似：
# <library path="lib/libdwa_local_planner">
#   <class name="dwa_local_planner/DWAPlannerROS" ...>
```

### Q4: 编译速度慢

**优化**:
```bash
# 只编译需要的包
catkin_make -DCATKIN_WHITELIST_PACKAGES="dwa_local_planner"

# 使用并行编译（根据 CPU 核心数）
catkin_make -j8 -l8

# 清理后重新编译
catkin_make clean
catkin_make -DCATKIN_WHITELIST_PACKAGES="dwa_local_planner"
```

## 修改 DWA 源码（可选）

### 修改源码后重新编译

```bash
# 修改文件
vim src/navigation/dwa_local_planner/src/dwa_planner.cpp

# 重新编译
catkin_make -DCATKIN_WHITELIST_PACKAGES="dwa_local_planner"

# 测试修改
roslaunch sim_nav nav.launch local_planner:=dwa_local_planner/DWAPlannerROS
```

### 添加调试输出

```cpp
// 在 dwa_planner.cpp 中添加
ROS_INFO_STREAM("DWA: Testing custom build!");
ROS_INFO_STREAM("DWA: max_vel_x = " << limits_.max_vel_x);

// 重新编译并查看输出
roslaunch sim_nav nav.launch local_planner:=dwa_local_planner/DWAPlannerROS
# 在终端中应该看到 ROS_INFO 输出
```

## 性能优化（源码级）

### 1. 调整采样策略

编辑 `src/navigation/dwa_local_planner/src/dwa_planner.cpp`:

```cpp
// 增加采样数量（默认 20）
const int vx_samples = 30;
const int vtheta_samples = 50;

// 或者在 YAML 中配置
```

### 2. 优化评价函数

编辑 `src/navigation/dwa_local_planner/src/dwa_planner.cpp`:

```cpp
// 修改权重（在 scoreTrajectory 函数中）
double DWAPlanner::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  cost += 40.0 * path_cost;     // 增加路径跟随权重
  cost += 20.0 * goal_cost;     // 降低目标权重
  cost += 0.02 * obstacle_cost; // 调整避障权重
  return cost;
}
```

### 3. 添加自定义约束

```cpp
// 在 generateTrajectories 函数中添加
if (v > max_vel_x * 0.8 && w > max_rot_vel * 0.8) {
  // 同时高速移动和转向会导致履带打滑，跳过
  continue;
}
```

## 编译配置对比

| 配置项 | 系统版本 (apt) | 源码编译 |
|--------|---------------|---------|
| **安装方式** | `apt-get install` | `catkin_make` |
| **版本** | 固定（ROS 发布时） | 最新（GitHub） |
| **可定制性** | ❌ 不可修改 | ✅ 可修改源码 |
| **调试支持** | ❌ 无源码调试 | ✅ 可加断点/日志 |
| **编译时间** | - | ~5-10 分钟 |
| **适用场景** | 生产环境 | 开发/研究 |

## 推荐开发流程

1. **初期开发**: 使用系统版本（快速迭代）
   ```bash
   sudo apt-get install ros-noetic-dwa-local-planner
   ```

2. **功能定制**: 从源码编译（修改算法）
   ```bash
   cd ~/robotcup2026
   catkin_make -DCATKIN_WHITELIST_PACKAGES="dwa_local_planner"
   ```

3. **生产部署**: 回到系统版本（稳定性）
   ```bash
   # 删除源码编译的版本
   rm -rf devel/lib/libdwa_local_planner.*
   
   # 使用系统版本
   sudo apt-get install ros-noetic-dwa-local-planner
   ```

## 参考资料

- [navigation 官方仓库](https://github.com/ros-planning/navigation)
- [DWA 算法文档](http://wiki.ros.org/dwa_local_planner)
- [Catkin 编译文档](http://wiki.ros.org/catkin/commands/catkin_make)
