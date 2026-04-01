25.11.20
目前已经完成视觉避障内容，但是目前存在问题点云消失太慢了
后续对 move_base 进行修改即可

构建地图过程中会出现消失的障碍物无法自动清除的问题，如果需要做到，可以更还建图节点，

25.12.1
视觉节点可用，local_costmap 的范围改为 8*8m //  26.1.27 已改回 4*4，并去掉了视觉点云补充

26.1.27
添加了 FAST_LIO 和 tare_planner 和 tare_navigation
FAST_LIO 不可用

26.3.3
重置了 FAST_LIO（已安装）和 tare_planner(未安装）
已经重新安装 fastlio，并补充了相关所缺，fastlio 无法通过一键安装依赖使用，需要根据 github 的官方文档和 catkin_make 的报错处理
目前 fastlio 直接使用官方提供的 config 和 launch，但出现仿真黑屏 gazebo 等报错的情况

26.3.4
目前 fastlio 在仿真中会出现全部无有效点云的情况，目前已经通过更改参数解决了崩溃问题，但仍然存在全部点云无效跳过全部帧率的问题
目前暂考虑为 gazebo 的雷达插件发布的数据包有问题导致 fastlio 无法正常发布数据，具体原因需要上实车后测试

当前版本编译过慢，且容易失败。请考虑通过精简 ws 包空间对减少编译负担

26.3.8
fastlio 的 gazebo 下崩溃是因为 ring 内存数组越界，尚未排除仿真插件问题

26.3.9
    3.9.0
        当日初始版本
    3.9.1
        版本目标：修复 gazebo 下 FAST_LIO 因数组越界导致的崩溃（Velodyne 分支 ring/time 风险）。

        本次改动：
        1) 修改 fast_lio/launch/mapping_velodyne.launch
           - 新增参数 gazebo_safe_mode，默认 true
           - 当 gazebo_safe_mode=true 时，强制覆盖：
             preprocess/lidar_type = 4（走 sim_handler）
             preprocess/timestamp_unit = 0

        2) 修改 fast_lio/config/velodyne.yaml
           - 补充 lidar_type=4（MARSIM/sim_handler）说明
           - 补充 gazebo_safe_mode 默认覆盖参数说明

        如何改回真实雷达：
        1) 启动时关闭安全模式
           roslaunch fast_lio mapping_velodyne.launch gazebo_safe_mode:=false
        2) 确认 fast_lio/config/velodyne.yaml
           - preprocess/lidar_type: 2   （Velodyne）
           - preprocess/timestamp_unit: 0（通常为秒）
           - common/lid_topic 与 imu_topic 改为实车实际话题

26.3.10
   3.10.0
      当日初始版本
   3.10.1
      版本目标：修正 FAST_LIO 在 Gazebo 中 IMU-LiDAR 外参坐标系不一致问题（继续使用 camera_imu_optical_frame）。

      本次改动：
      1) 修改 fast_lio/config/velodyne.yaml 外参参数
         - 明确外参定义为：从 IMU link 到 LiDAR link，即
           camera_imu_optical_frame -> lidar
         - mapping/extrinsic_T: [-0.002761, -0.071870, -0.099400]
         - mapping/extrinsic_R:
           [0, -1, 0,
            0,  0, -1,
            1,  0,  0]

      说明：
      - 以上外参由 URDF 固定变换链计算得到，避免把 base_link -> lidar 误当作 IMU -> lidar 使用。
   3.10.2
      版本目标：记录本轮 URDF 改动与 IMU 频率调整，统一 FAST_LIO 主里程计配置说明。

      本次改动（URDF）：
      1) 关闭 Gazebo 差速里程计发布（differential_drive 插件）
         - 原来：
           <publishOdom>true</publishOdom>
           <publishOdomTF>true</publishOdomTF>
         - 现在：
           <publishOdom>false</publishOdom>
           <publishOdomTF>false</publishOdomTF>

      2) IMU 插件 frameName 调整
         - 原来：
           <frameName>camera_imu_optical_frame</frameName>
         - 现在：
           <frameName>camera_imu_frame</frameName>

      3) IMU 插件频率调整（用户手动修改）
         - 原来：
           <updateRate>200.0</updateRate>
           <updateRateHZ>200.0</updateRateHZ>
         - 现在：
           <updateRate>100.0</updateRate>
           <updateRateHZ>100.0</updateRateHZ>

      说明：
      - 第 3 项为本次手动调整，已纳入版本记录。
   3.10.3
      版本目标：统一 launch 层 TF 链，修复 RViz 在 camera_init 视角下车体"立起来"与 TF 树不一致问题。

      改动文件：
      - car/launch/gazebo.launch

      参数对比（原来 -> 现在）：
      1) body -> base_footprint 静态 TF
         - args="-0.003760 0.228831 -0.254899 -0.5 0.5 -0.5 -0.5 body base_footprint 40"
         - args="-0.254900 0.003761 -0.228830 0 0 0 body base_footprint 50"

      2) base_footprint -> base_link 静态 TF 发布频率
         - args="0 0 0 0 0 0 base_footprint base_link 40"
         - args="0 0 0 0 0 0 base_footprint base_link 50"

      解决效果：
      - 修复 camera_init 下车体姿态异常，显示与 base_link 观察结果一致。
      - 将 FAST_LIO 主链与机器人本体链对齐，TF 主干统一为：
        `camera_init -> body -> base_footprint -> base_link -> sensors/wheels`
      - 消除原先链路对齐不完整导致的 TF 语义不一致问题。

      影响说明：
      - 本次仅为 launch 层 TF 关系与发布参数修正，不改 FAST_LIO 建图算法本身。

26.3.11
   3.11.0
      当日初始版本
   3.11.1
      版本目标：恢复 Gazebo 下 FAST_LIO 的 Velodyne 主流程，并完成 URDF 与 FAST_LIO 参数对齐检查。

      本次改动：
      1) 修改 fast_lio/launch/mapping_velodyne.launch
         - gazebo_safe_mode 默认值：true -> false
         - 含义：默认走 Velodyne 分支（preprocess/lidar_type=2），不再默认强制切到 sim_handler(lidar_type=4)

      2) 修改 fast_lio/launch/mapping_velodyne.launch
         - cube_side_length：1000 -> 100
         - 含义：缩小局部地图立方体边长，减少不必要的大范围地图维护开销

      3) 修改 car/urdf/car.urdf
         - Velodyne 垂直扫描参数：<vertical><resolution>2</resolution> -> <resolution>1</resolution>
         - 含义：与 VLP-16（16 线）语义一致，避免垂直线束配置歧义

      对齐检查结论（本轮核对）：
      - 话题已对齐：
        FAST_LIO common/lid_topic=/velodyne_points，common/imu_topic=/camera/imu
        URDF 插件发布 topicName=velodyne_points，IMU topicName=/camera/imu
      - 频率/线数已对齐：
        FAST_LIO scan_line=16、scan_rate=10
        URDF lidar 为 16 线、10Hz
      - 当前默认启动即为 Velodyne 组配置；仅在手动设置 gazebo_safe_mode:=true 时才切换到仿真安全分支

  3.11.2
      现在有扫描数据了，但是 fastlio 导致零飘，且无法建图

26.3.13
   3.13.0
      当日原始版本
   3.13.1
      删除掉了 mappinglaunch 的安全模式
   3.13.2
      完全重置了 urdf 的雷达和视觉插件
   3.13.3
      fastlio 乱漂的原因是雷达插件点云过少导致无法定位
      通过 13.2 重置插件解决了
   3.13.4
      更改了 fastlio 的源码，开启了地图发布（点云累计）

26.3.14
   3.14.0
      当日原始版本，取消了点云累积
   3.14.1
      重新加入点云累积，修改了 fastlio 的 launch 文件优化了点云数据的累积效果，如果不做地形检测需要将地面点云滤掉，暂时不考虑滤波

26.3.30 - V5.7 救援模式重大修复 (Tag: 262.401.1)
   版本目标：修复救援模式中的严重 Bug，提升救援模式稳定性和准确性

   【严重 Bug 修复】
   1. 救援成功后立即再次触发救援
      - 修复 finalizeCancel() 中未清零 consecutive_plan_failures
      - 修复 stuck_abort_count 和位移检查状态未清零
      - 影响：救援成功后会立即再次触发救援模式

   2. 位移检查逻辑混乱
      - 修复第一次 abort 时就进行位移比较
      - 改为第一次只记录位置，后续才比较位移
      - 影响：位移检查准确性大幅提升

   3. 救援后第一个目标被误判
      - 修复救援模式退出时未清零障碍物检测状态
      - 添加 first_obstacle_detect_time 清零逻辑
      - 影响：避免救援后第一个目标被误判为有障碍物

   【核心功能修复】
   4. 黑名单超时检查逻辑
      - 从统一时间检查改为使用 cool_until 逐点检查
      - 救援模式期间暂停黑名单计时，退出时补偿时间
      - 影响：黑名单时间计算准确，不会因救援模式而丢失计时

   5. rescue_goal_start_time 计时时机
      - 从进入救援模式时启动改为 attemptRescueGoal 时启动
      - 影响：救援目标超时计算准确

   6. goal_start_time 在救援模式下跳过设置
      - 救援模式使用独立计时器，不干扰正常模式
      - 影响：救援模式与正常模式计时完全解耦

   7. last_stuck_abort_check_time 显式清零
      - 所有救援模式退出场景都清零位移检查状态
      - 影响：下次进入救援模式时位移检查从头开始

   【新增功能】
   8. shrink 安全检测像素范围可配置
      - 新增 global_safe_window_pixels 参数（默认 1=3x3）
      - 支持 2=5x5, 3=7x7 等配置
      - 影响：可根据地图分辨率灵活调整检测范围

   9. 位移计算封装为 checkDisplacement() 函数
      - 消除重复代码，提高可维护性
      - 自动更新参考位置，避免忘记更新

   【代码优化】
   - 清理所有调试注释标记（【新增】【修复】等）
   - 统一注释风格
   - 优化日志输出格式

   【配置文件】
   - rrt_goal_decision.launch: 添加 V5.7 参数说明
   - 新增 test_global_costmap.launch: 全局代价地图测试工具
   - 新增 global_costmap_tester.cpp: 代价地图可视化工具
