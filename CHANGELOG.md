# RobotCup 2026 项目更新日志

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
