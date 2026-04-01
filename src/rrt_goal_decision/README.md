# rrt_goal_decision - V5.7 救援模式重大修复

**Tag**: 262.401.1  
**日期**: 26.3.30

---

## 版本目标

修复救援模式中的严重 Bug，提升救援模式稳定性和准确性。

---

## 严重 Bug 修复

### 1. 救援成功后立即再次触发救援
- **问题**: finalizeCancel() 中未清零 consecutive_plan_failures、stuck_abort_count 和位移检查状态
- **修复**: 救援模式退出时清零所有计数器
- **影响**: 救援成功后不会立即再次触发救援模式

### 2. 位移检查逻辑混乱
- **问题**: 第一次 abort 时就进行位移比较
- **修复**: 第一次只记录位置，后续才比较位移
- **影响**: 位移检查准确性大幅提升

### 3. 救援后第一个目标被误判
- **问题**: 救援模式退出时未清零障碍物检测状态
- **修复**: 添加 first_obstacle_detect_time 清零逻辑
- **影响**: 避免救援后第一个目标被误判为有障碍物

---

## 核心功能修复

### 4. 黑名单超时检查逻辑
- **改动**: 从统一时间检查改为使用 cool_until 逐点检查
- **新增**: 救援模式期间暂停黑名单计时，退出时补偿时间
- **影响**: 黑名单时间计算准确，不会因救援模式而丢失计时

### 5. rescue_goal_start_time 计时时机
- **改动**: 从进入救援模式时启动改为 attemptRescueGoal 时启动
- **影响**: 救援目标超时计算准确

### 6. goal_start_time 在救援模式下跳过设置
- **改动**: 救援模式使用独立计时器，不干扰正常模式
- **影响**: 救援模式与正常模式计时完全解耦

### 7. last_stuck_abort_check_time 显式清零
- **改动**: 所有救援模式退出场景都清零位移检查状态
- **影响**: 下次进入救援模式时位移检查从头开始

---

## 新增功能

### 8. shrink 安全检测像素范围可配置
- **新增参数**: `global_safe_window_pixels`（默认 1=3x3）
- **支持配置**: 2=5x5, 3=7x7 等
- **影响**: 可根据地图分辨率灵活调整检测范围

### 9. 位移计算封装为 checkDisplacement() 函数
- **改动**: 消除重复代码，提高可维护性
- **新增**: 自动更新参考位置，避免忘记更新

---

## 代码优化

- 清理所有调试注释标记（【新增】【修复】等）
- 统一注释风格为 V5.7 版本号
- 优化日志输出格式

---

## 配置文件

- **rrt_goal_decision.launch**: 添加 V5.7 参数说明
- **新增**: test_global_costmap.launch - 全局代价地图测试工具
- **新增**: global_costmap_tester.cpp - 代价地图可视化工具

---

## 使用示例

```bash
# 启动 rrt_goal_decision
roslaunch rrt_goal_decision rrt_goal_decision.launch

# 测试全局代价地图（可选）
roslaunch rrt_goal_decision test_global_costmap.launch
```

---

## 参数说明

### shrink 检测参数
```xml
<!-- shrink 安全检测像素范围（半径） -->
<!-- 1=3x3, 2=5x5, 3=7x7 -->
<param name="global_safe_window_pixels" value="1" />
```

### 救援模式参数
```xml
<!-- 触发救援的连续 plan failed 次数 -->
<param name="rescue_trigger_failures" value="5" />

<!-- 触发救援的 abort 累计次数（stuck 不计入） -->
<param name="rescue_trigger_abort_count" value="10" />

<!-- 单个救援目标的容忍时间（秒） -->
<param name="rescue_goal_timeout" value="15.0" />

<!-- 脱困最大尝试次数（安全保护机制） -->
<param name="rescue_max_attempts" value="20" />
```

---

## 版本历史

- **V5.7** (26.3.30): 救援模式重大修复
- **V5.6**: 智能救援与位移检查
- **V5.5**: 单目标超时控制
- **V5.4**: 双触发机制（连续 failed/stuck 计数）
- **V5.3**: 基于失败次数的救援模式
