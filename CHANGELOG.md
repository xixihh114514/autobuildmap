# RobotCup 2026 项目更新日志

## 26.4.7 - V5.7.3 - 架构重构与逻辑优化

### 架构重构 (工程化)
1. **分离代码结构**
   - 将庞大的 `rrt_goal_node.cpp` 拆分为标准的 `main()` 入口、类头文件 (`frontier_goal_manager.h`) 和实现文件 (`.cpp`)
2. **提取工具函数**
   - 将通用函数 `worldToMapCoords` (坐标转换) 和 `checkDisplacement` (位移检查) 提取到 `frontier_utils.h/cpp` 中
3. **参数管理重构**
   - 提取了 30+ 个 ROS 参数到 `frontier_params.h/cpp`，定义 `FrontierParams` 结构体统一管理，类内部只保留一个 `params` 实例，代码更整洁

### 核心逻辑修复
1. **修复救援模式误退出**
   - 解决了救援模式激活瞬间，因旧目标的 Cancel 确认触发 `finalizeCancel()` 导致立即重置状态退出的 Bug
2. **修复权重归一化 Bug**
   - 修复了 `num_weight` 归一化时 `max_num` 包含无效（被过滤）Cluster 导致有效点权重过小的问题
   - 改为“先过滤，再计算 max"的两遍扫描逻辑
3. **修复坐标映射偏移**
   - 将所有地图坐标转网格的逻辑从 `static_cast<int>`（截断）改为 `std::round`（四舍五入），半径计算改为 `std::ceil`，解决了圆心对齐误差
4. **已访问点清理**
   - 目标成功后直接从 `frontier_buffer` 中删除对应的点，不再只是加入黑名单，避免内存浪费和重复扫描

### 救援触发器统一
1. **重命名计数器**
   - 将 `stuck_abort_count` 更名为 `rescue_trigger_count`，明确其统计三种情况的含义
2. **新增 Timeout 触发**
   - 将 **Goal Total Timeout**（目标总超时）也纳入救援触发条件
3. **函数抽象**
   - 提取了 `incrementRescueTriggerCount(reason)` 函数，统一处理三种触发源（ABORTED, Stuck, Timeout）的位移检查和计数逻辑，消除了大量重复代码

---

## 26.4.3 - V5.7.2 - 代码重构与归一化修复

### 代码重构
1. **消除重复代码，提取 5 个辅助函数**
   - `resetRescueModeState()` — 统一重置救援模式所有状态（消除 3 处重复）
   - `computeRescueCenter()` — 统一计算救援中心坐标（消除 3 处重复）
   - `activateRescueMode()` — 统一进入救援模式的完整流程（消除 2 处重复）
   - `worldToMapCoords()` — 统一世界坐标到地图网格转换（消除 4 处重复）
   - `markGoalAsFailed()` — 统一标记目标失败并加入黑名单（消除 5 处重复）

2. **重构涉及的函数**
   - `finalizeCancel()` — 使用新辅助函数
   - `monitorGoal()` — 5 处失败记录改用 `markGoalAsFailed()`
   - `spin()` — 两处救援触发改用 `activateRescueMode()`
   - `attemptRescueGoal()` — 使用 `computeRescueCenter()`
   - `getCost()` / `checkNearbyObstacleRunning()` / `isNearbyCostSafe()` — 使用 `worldToMapCoords()`

### Bug 修复
3. **修复救援模式刚进入就退出的问题**
   - 根因：`activateRescueMode()` 设置 `is_rescue_mode = true` 后，前一个 goal 的 cancel 确认进入 `finalizeCancel()`，误判为救援完成并调用 `resetRescueModeState()`
   - 修复：移除 `finalizeCancel()` 中的救援模式重置逻辑，只在救援成功或达到最大尝试次数时重置

4. **修复 `num_weight` 归一化值过小的问题**
   - 根因：`max_num` 在所有 cluster 中计算，包括被 shrink_fail/obstacle_fail/blacklist_fail 过滤掉的超大 cluster，导致 valid cluster 的 `raw_num` 被严重压缩
   - 修复：改为两遍扫描，先过滤出 valid cluster，再在其中计算 `max_num`，确保归一化基准正确

5. **修复地图坐标转换的圆心对齐问题**
   - 根因：`static_cast<int>` 向下取整导致物理圆心偏移到相邻像素
   - 修复：所有世界坐标到地图网格的转换改用 `std::round` 四舍五入
   - 涉及函数：`worldToMapCoords()`、`shrinkOnGlobalCostmap()`（3 处）、`calculateUnknownProportion()`、`calculateFrontierOrientation()`

6. **修复搜索半径被截断缩小的问题**
   - 根因：`static_cast<int>(radius / res)` 向下取整导致搜索范围缩小
   - 修复：`calculateUnknownProportion()` 和 `calculateFrontierOrientation()` 中的半径计算改用 `std::ceil` 向上取整

7. **删除未使用的变量 `select_time`**

### 代码清理
8. **移除 `selectGoal()` 中的调试统计变量**
   - 删除 `total_points`、`shrink_fail`、`obstacle_fail`、`blacklist_fail`、`valid_count`
   - 简化过滤逻辑，保留核心功能

---

## 26.4.1 - V5.7.1 - 死胡同脱困优化

### 性能优化
1. **救援模式激活流程优化**
   - 修复救援模式激活时未重置 `just_canceled` 的问题
   - 确保救援尝试不会被跳过
   - **显著提升机器人在死胡同内的脱困能力**

2. **全局路径平滑优化**
   - 引入路径平滑算法，优化大角度转弯处的路径
   - **减少机器人在弯道处的调整次数**
   - 提升导航流畅性和效率

### 已知问题
- **连续弯道中仍有概率需要大量调整，会导致误判 stuck**
  - 原因：连续弯道中机器人速度降低，被误判为 stuck
  - 临时方案：适当提高 `rescue_trigger_stuck_abort_count` 阈值
  - 待优化：改进 stuck 检测逻辑，考虑弯道场景

### V5.7 历史修复
2. **救援成功后立即再次触发救援**
   - 修复 `finalizeCancel()` 中未清零计数器的问题
   - 救援模式退出时清零所有状态

3. **位移检查逻辑混乱**
   - 第一次 stuck 只记录位置，不计数
   - 后续 stuck 对比位移，超过 0.5m 清零计数

4. **救援后第一个目标被误判**
   - 救援模式退出时清零障碍物检测状态

5. **黑名单超时检查逻辑**
   - 从统一时间检查改为使用 `cool_until` 逐点检查
   - 救援模式期间暂停黑名单计时，退出时补偿时间

6. **rescue_goal_start_time 计时时机**
   - 从进入救援模式时启动改为 `attemptRescueGoal` 时启动

7. **goal_start_time 在救援模式下跳过设置**
   - 救援模式使用独立计时器

8. **last_stuck_abort_check_time 显式清零**
   - 所有救援模式退出场景都清零位移检查状态

### 重要变更
9. **恢复 stuck/abort 共同计数**
   - stuck 和 abort 共享 `stuck_abort_count` 计数器
   - 宁可错判也不让机器人彻底卡死
   - 第一次 stuck 检测：记录位置，不计数
   - 后续 stuck 检测：对比位移，>0.5m 清零，否则计数 +1

### 新增功能
9. **shrink 安全检测像素范围可配置**
   - 新增 `global_safe_window_pixels` 参数（默认 1=3x3）
   - 支持 2=5x5, 3=7x7 等配置

10. **位移计算封装**
    - 使用 `std::hypot()` 计算位移

### 新增工具
11. **path_optimizer 路径优化器**
    - 基于梯度下降的路径平滑
    - 自动远离高代价区域
    - 性能优化：10-50ms 完成优化

### 参数说明
```yaml
# 救援模式触发参数
rescue_trigger_failures: 5              # 连续 plan failed 次数
rescue_trigger_stuck_abort_count: 10    # stuck/abort 累计次数
rescue_goal_timeout: 15.0               # 单个救援目标超时（秒）
rescue_max_attempts: 20                 # 最大救援尝试次数

# shrink 检测参数
global_safe_window_pixels: 1            # 像素范围（1=3x3, 2=5x5, 3=7x7）

# 路径优化器参数
path_optimizer/cost_threshold: 40.0     # 低代价阈值
path_optimizer/slice_interval: 0.4      # 切片间隔（米）
path_optimizer/smooth_window: 7         # 平滑窗口大小
```

---

## 历史版本

### V5.6 (26.3.30) - 智能救援与位移检查
- 添加 stuck 位移检查
- 救援模式独立计时器
- 黑名单时间补偿机制

### V5.5 (26.3.29) - 单目标超时控制
- 添加 `rescue_goal_timeout` 参数
- 单个救援目标超时后尝试新目标

### V5.4 (26.3.28) - 双触发机制
- 添加 `rescue_trigger_stuck_abort_count` 参数
- stuck 和 abort 共同触发救援模式

### V5.3 (26.3.27) - 基于失败次数的救援模式
- 添加 `rescue_trigger_failures` 参数
- 连续 plan failed 触发救援

---

## 快速开始

```bash
# 编译
cd ~/robotcup2026
catkin_make

# 启动导航
roslaunch sim_nav nav.launch

# 启动路径优化器（可选）
roslaunch path_optimizer path_optimizer_with_teb.launch
```

## 文档

- [rrt_goal_decision 使用说明](src/rrt_goal_decision/README.md)
- [path_optimizer 使用说明](src/path_optimizer/README.md)
