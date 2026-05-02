# src 工作区说明

这个 `src` 目录是 RoboCup2026 自主探索 / 建图工作区的 ROS 1 `catkin` 源码区，当前已经同时放入了机器人模型、仿真、FAST_LIO、TARE、本地点云处理和若干导航相关依赖包。

## 当前主链路

- `rebo24`：当前默认机器人模型与 Gazebo 启动入口。
- `fast_lio`：当前主定位 / 点云里程计链路。
- `loam_interface`：把 FAST_LIO 的里程计与点云桥接到后续探索链路。
- `terrain_analysis` 与 `terrain_analysis_ext`：地形分析与扩展地形图。
- `local_planner`：局部路径搜索与底盘跟踪控制。
- `sensor_scan_generation`：为 TARE 补充传感器扫描输入。
- `tare_planner`：探索规划主包。
- `hector_slam`：二维地图绘制与导出。
- `image_map_gazebo`：根据图片网格快速生成 Gazebo 地图场景的新包。

## 推荐启动顺序

当前仓库里已经验证过的一套主流程如下，基本沿用 2026-03-23 的记录：

1. 启动仿真底盘
   `roslaunch rebo24 simbase.launch`
2. 启动 FAST_LIO
   `roslaunch fast_lio mapping_velodyne.launch`
3. 启动里程计桥接
   `roslaunch loam_interface loam_interface.launch`
4. 启动地形链路
   `roslaunch terrain_analysis_ext sim_terrain.launch`
5. 启动局部规划
   `roslaunch local_planner local_planner.launch`
6. 启动扫描生成
   `roslaunch sensor_scan_generation sensor_scan_generation.launch`
7. 启动 TARE 探索
   `roslaunch tare_planner explore_robocup.launch`

如果要跑车库参数，可把最后一步替换为：

```bash
roslaunch tare_planner explore_garage.launch
```

## 编译建议

当前工作区包很多，历史上已经多次出现编译慢、依赖多、整体构建容易失败的问题。日常建议优先按包编译：

```bash
cd /home/lzk/robotcup2026
catkin build rebo24 fast_lio loam_interface terrain_analysis terrain_analysis_ext local_planner sensor_scan_generation tare_planner image_map_gazebo
source devel/setup.bash
```

如果 `fast_lio` 缺依赖，不要只依赖一键安装脚本，通常还是要结合官方文档和本地报错逐项补齐。

## image_map_gazebo 包

`image_map_gazebo` 用于把图片网格地图快速转成 Gazebo world，适合先做场景验证再接入自主探索链路。

地图约定：

- 1 个图片方格 = 1.0 m
- 图片上方 = Gazebo +Y
- 图片右方 = Gazebo +X
- 黑色线段 = 墙体
- “箱子” = 木箱障碍物
- “斜坡 / 楼梯 / 操作台” = 对应实体模型
- “-” 和 `|` = 低矮路障
- “门口” = 绿色入口标记，无碰撞

启动方式：

```bash
roslaunch image_map_gazebo image_map.launch
```

重新生成 world：

```bash
cd /home/lzk/robotcup2026/src/image_map_gazebo
/usr/bin/python3 scripts/generate_image_map_world.py
```

如需改比例，修改 `scripts/generate_image_map_world.py` 里的 `CELL`。

## 当前已知问题

- 视觉避障链路已做过一轮接入，但历史上存在障碍点云消失慢的问题。
- FAST_LIO 在仿真里曾多次出现崩溃、无效点云、零飘和无法稳定建图的问题，是否彻底解决仍需要继续验证。
- 当前 TARE 依赖滤波、LOAM 接口、本地规划、地形分析、扫描生成和探索节点一起配合，地形节点不能简单全关。
- `local_planner` 最近一次记录仍在做诊断性调参，重点是窄通道和 90 度转弯卡住问题。

## 版本记录

### 26.5.2

- 新增 `image_map_gazebo` 包，用于图片地图转 Gazebo world。
- 重写 `src/readme.md`，补充工作区说明、推荐启动顺序和当前链路说明。
- `rebo24/launch/gazebo.launch` 改为直接加载 `image_map_gazebo/worlds/robocup2026_real.world`，并在 launch 内统一设置仿真出生点与朝向。
- `image_map_gazebo/worlds/robocup2026_real.world` 清理了 world 内遗留的车模状态，并把地图整体按当前车体相对关系做了位置修正，便于直接配合 `rebo24` 启动。
- `tare_planner` 新增分支锚点恢复与卡死检测恢复：
  - 支持“第一条支路探完后回到分叉，再继续第二条、第三条”的前进式回撤。
  - 连续低进展会触发 `stuck_recovery_mode`，并向下游发布 `/stuck_recovery_mode`。
  - 当前默认阈值为：`kRecoveryProgressMinDist=0.15 m`，`kStuckCycleThreshold=4`，规划检测周期约 `1 Hz`，因此约 4 秒进入卡死脱困判断。
- `local_planner` 补充了窄道 / 死胡同脱困控制：
  - 默认仍然保持前进优先，不允许普通路径跟踪时长距离倒车。
  - 进入 `stuck_recovery_mode` 后，仅允许短倒车脱困，当前默认 `stuckReverseDist=0.12 m`，`stuckReverseSpeed=0.08 m/s`。
  - 当恢复目标已经落到车后方时，优先“停住后原地掉头”，不再继续沿原前向路径慢慢顶墙。
  - 新增 `turnaroundStopAngle`、`turnaroundReverseTriggerAngle`、`turnaroundResumeAngle` 3 个参数，便于后续直接在 launch 里调原地掉头与短倒车触发条件。

### 历史原始记录

25.11.20
目前已经完成视觉避障内容，但是目前存在问题点云消失太慢了
后续对move_base进行修改即可

构建地图过程中会出现消失的障碍物无法自动清除的问题，如果需要做到，可以更还建图节点、

25.12.1
视觉节点可用，local_costmap的范围改为8*8m //  26.1.27已改回4*4，并去掉了视觉点云补充

26.1.27
添加了FAST_LIO和tare_planner和tare_navigation
FAST_LIO不可用

26.3.3
重置了FAST_LIO（已安装）和tare_planner(未安装）
已经重新安装fastlio，并补充了相关所缺，fastlio无法通过一键安装依赖使用，需要根据github的官方文档和catkin_make的报错处理
目前fastlio直接使用官方提供的config和launch，但出现仿真黑屏gazebo等报错的情况

26.3.4
目前fastlio在仿真中会出现全部无有效点云的情况，目前已经通过更改参数解决了崩溃问题，但仍然存在全部点云无效跳过全部帧率的问题
目前暂考虑为gazebo的雷达插件发布的数据包有问题导致fastlio无法正常发布数据，具体原因需要上实车后测试

当前版本编译过慢，且容易失败。请考虑通过精简ws包空间对减少编译负担

26.3.8
fastlio的gazebo下崩溃是因为ring内存数组越界，尚未排除仿真插件问题

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
      版本目标：统一 launch 层 TF 链，修复 RViz 在 camera_init 视角下车体“立起来”与 TF 树不一致问题。

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
         - 含义：与 VLP-16（16线）语义一致，避免垂直线束配置歧义

      对齐检查结论（本轮核对）：
      - 话题已对齐：
        FAST_LIO common/lid_topic=/velodyne_points，common/imu_topic=/camera/imu
        URDF 插件发布 topicName=velodyne_points，IMU topicName=/camera/imu
      - 频率/线数已对齐：
        FAST_LIO scan_line=16、scan_rate=10
        URDF lidar 为 16 线、10Hz
      - 当前默认启动即为 Velodyne 组配置；仅在手动设置 gazebo_safe_mode:=true 时才切换到仿真安全分支

  3.11.2
      现在有扫描数据了，但是fastlio导致零飘，且无法建图

26.3.13
   3.13.0
      当日原始版本
   3.13.1
      删除掉了mappinglaunch的安全模式
   3.13.2
      完全重置了urdf的雷达和视觉插件
   3.13.3
      fastlio乱漂的原因是雷达插件点云过少导致无法定位
      通过13.2重置插件解决了
   3.13.4
      更改了fastlio的源码，开启了地图发布（点云累计）

26.3.14
   3.14.0
      当日原始版本，取消了点云累积
   3.14.1
      重新加入点云累积，修改了fastlio的launch文件优化了点云数据的累积效果,如果不做地形检测需要将地面点云滤掉

26.3.16
   3.16.0
      当日初始版本
   3.16.1
      自写包加入滤波，已经完全过滤掉地面点云，可被后续节点调用

26.3.17
   3.17.0
      当日初始版本
   3.17.1
      修改了滤波后的点云的数量
   3.17.2
      加入了tare_planner

26.3.18
   3.18.0
      当日初始版本，给tare加上了注释

26.3.19
   3.19.0
   跑通了tare，但是很睿智，一直😎墙
   解决了雷达扫描范围错误，重置了滤波效果，当前效果很好

26.3.20
   3.20.0
      修复了点云雷达问题，已经验证当前包必须开地形检测，否则无法运行

26.3.23
   3.23.0
      目前tare需要的节点有滤波、loam、local_planner、地形检测两个节点（需要关闭地形检测，但是不能完全关掉两个点）、sensor_scan、explore（参数在garage）
      新增了快速启动脚本，完整启动路径为先分别启动simbase，然后启动sim_fast_lio，然后启动loam，然后启动sim_terrain,然后启动local_planner，再然后启动sensor_scan,最后启动explore

   3.25.0
      更改了滤波方式，使用pclros的corpbox来过滤车体点云

   3.28.0
      当日初始版本
   3.28.1
      local中允许车辆倒车

   3.32.1
      解决了嵌套git的问题

26.3.30
   3.30.0
      当日初始版本
   3.30.1
      local_planner先做诊断性调参，验证窄通道和90度弯卡住是否主要由方向约束过强导致
      修改了local_planner.launch：
      checkRotObstacle：true -> false
      dirWeight：0.02 -> 0.005
      pathCropByGoal：true -> false
      备注：dirThre原始默认值为90.0，当前180.0为手动调整，不属于本次代改内容
   3.30.2
      针对窄路丁字路口继续调local_planner
      修改了local_planner.launch：
      twoWayDrive：false -> true
      lookAheadDis：1.5 -> 0.7
      说明：允许倒车修正姿态，并缩短前视距离，避免局部跟踪在丁字路口和直角弯处切不进去
   3.30.3
      针对窄通道内来回振荡继续调local_planner
      修改了local_planner.launch：
      dirWeight：0.01 -> 0.02
      dirThre：140.0 -> 120.0
      lookAheadDis：0.7 -> 0.9
      yawRateGain：7.5 -> 5.0
      stopYawRateGain：7.5 -> 5.0
      switchTimeThre：1.0 -> 2.0
      说明：增强目标方向约束，减小转向增益，拉长前后切换时间，尽量降低窄通道内左右摆动和前后反复
   3.30.4
      振荡幅度减小但仍无法正常通过窄通道，继续做折中调参
      修改了local_planner.launch：
      autonomySpeed：1.0 -> 0.7
      dirWeight：0.02 -> 0.015
      dirThre：120.0 -> 130.0
      lookAheadDis：0.9 -> 0.8
      说明：在抑制振荡的基础上适当放宽通过性，并降低自动速度
   3.30.5
      回退到会在通道中振荡的版本
      修改了local_planner.launch：
      autonomySpeed：0.7 -> 1.0
      dirWeight：0.015 -> 0.02
      dirThre：130.0 -> 120.0
      lookAheadDis：0.8 -> 0.9
      说明：回退到3.30.3对应的参数组，保留twoWayDrive=true、yawRateGain=5.0、stopYawRateGain=5.0、switchTimeThre=2.0
   3.30.6
      继续回退到振荡更明显的版本
      修改了local_planner.launch：
      dirWeight：0.02 -> 0.01
      dirThre：120.0 -> 140.0
      lookAheadDis：0.9 -> 0.7
      yawRateGain：5.0 -> 7.5
      stopYawRateGain：5.0 -> 7.5
      switchTimeThre：2.0 -> 1.0
      说明：回退到3.30.2对应的参数组，保留twoWayDrive=true、autonomySpeed=1.0

26.3.31
   3.31.0
      当日初始版本
   3.31.1
      版本目标：新增小尺度室内地图探索 profile，统一当前手动启动流程，给 `robotcup_map` 这类窄通道/直角弯地图准备一套独立的 launch 与 TARE 参数入口。

      本次改动：
      1) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - 将局部规划关键参数提升为 launch arg，便于后续继续调参
         - 默认值改为小地图 profile：
           `twoWayDrive=true`
           `autonomySpeed=0.7`
           `adjacentRange=3.2`
           `dirWeight=0.005`
           `dirThre=160.0`
           `pathCropByGoal=false`
           `minPathScale=0.6`
           `lookAheadDis=0.6`
           `yawRateGain=6.0`
           `stopYawRateGain=6.0`
           `switchTimeThre=1.2`

      2) 修改 `autonomous_exploration_development_environment/src/terrain_analysis_ext/launch/sim_terrain.launch`
         - `checkTerrainConn` 默认值：`true -> false`
         - 含义：小尺度室内图默认关闭连通性约束，避免可通行区域被过早裁掉

      3) 新增 `tare_planner/config/robotcup_indoor.yaml`
         - 基于 `indoor.yaml` 派生独立小地图探索配置
         - 缩短 waypoint 外延与 lookahead
         - 减小 frontier 最小点数
         - 缩小视点碰撞边距
         - 调整视点分辨率、传感器范围和覆盖膨胀半径

      4) 新增 `tare_planner/launch/explore_robotcup_indoor.launch`
         - 固定 `scenario=robotcup_indoor`
         - 保留 `rviz`、`rosbag_record`、`use_boundary` 接口

      5) 修改 `src/readme.md`
         - 改成当前这套手动启动流程说明
         - 固定推荐顺序为：
           `simbase -> mapping_velodyne -> vehicle_cropbox_filter -> loam_interface -> sim_terrain -> local_planner -> sensor_scan_generation -> explore_robotcup_indoor`
         - 保留原始历史记录，方便后续回溯

      当前验证结论：
      - 上游链路已打通：`/registered_scan`、`/terrain_map`、`/terrain_map_ext`、`/way_point`、`/path`、`/cmd_vel` 都能正常建立
      - 当前仍存在待修问题：
        1) `explore` 在部分地图上一进入就判断探索完成
        2) `/path` 被 `FAST_LIO` 和 `local_planner` 同时发布，存在话题冲突
      - 以上待修问题不属于本次版本已完成内容，后续继续处理

26.4.7
   4.7.1
      版本目标：local_planner先做第一步调参，默认关闭倒车，优先压住丁字路口前后切换和左右振荡。

      本次改动：
      1) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - `twoWayDrive`：`true -> false`
         - `dirWeight`：`0.005 -> 0.01`
         - `dirThre`：`160.0 -> 120.0`
         - 新增 launch arg：`pathRangeBySpeed=false`
         - 新增 launch arg：`dirDiffThre=0.2`
         - `localPlanner/pathRangeBySpeed`：`true -> false`
         - `pathFollower/dirDiffThre`：`0.1 -> 0.2`

      2) 修改 `src/readme.md`
         - 更新当前推荐用途
         - 更新 `local_planner.launch` 当前默认值说明
         - 追加本轮第一步调参记录

      说明：
      - 本轮按当前需求不保留倒车修正能力，先观察丁字路口是否停止来回换向
      - 1.2m 直角弯的通过性暂不在本轮处理，后续再做第二步
   4.7.2
      版本目标：重置 TARE 的全部前置节点启动链路，并将 `loam_interface` 并入 `terrain` 的一键启动入口，减少手动分段启动。

      本次改动：
      1) 修改 `autonomous_exploration_development_environment/src/terrain_analysis_ext/launch/sim_terrain.launch`
         - 统一为地形链路一键启动入口
         - 启动顺序调整为：
           `loam_interface.launch` 立即启动
           `terrain_analysis.launch` 延迟 5 秒启动
           `terrain_analysis_ext.launch` 延迟 10 秒启动
         - `terrain_analysis_ext` 的 `checkTerrainConn` 固定为 `false`

      2) 修改 `autonomous_exploration_development_environment/src/terrain_analysis/launch/terrain_analysis.launch`
         - 新增 `delay_sec` 接口
         - 支持被组合 launch 调用时延迟启动

      3) 修改 `autonomous_exploration_development_environment/src/terrain_analysis_ext/launch/terrain_analysis_ext.launch`
         - 保留 `checkTerrainConn` 接口
         - 新增 `delay_sec` 接口
         - 支持被组合 launch 调用时延迟启动

      4) 修改 `src/readme.md`
         - 追加本轮启动链路重置记录

      当前前置节点说明：
      - 本轮将 TARE 使用到的前置节点重新整理为：
        `vehicle_cropbox_filter -> sim_terrain(内含 loam_interface + terrain_analysis + terrain_analysis_ext) -> local_planner -> sensor_scan_generation`
      - 其中 `sim_terrain` 现在已经包含 `loam_interface`，不需要再单独手动启动 loam

      当前用法：
      - 地形链路直接运行：
        `roslaunch terrain_analysis_ext sim_terrain.launch`

      说明：
      - 本轮重点是整理和重置 TARE 上游前置节点的启动方式，不涉及 TARE 本体参数修改

26.4.7
   4.7.3
      版本目标：补充当前 local_planner 版本记录，统一说明本轮已确认内容，并重点记录当前怀疑参数 `useTerrainAnalysis`。

      本次改动：
      1) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - 按功能对参数重新分组
         - 补充逐项中文注释
         - 修正 XML 注释格式，保证 launch 文件可正常解析
         - 当前已确认默认外参：
           `sensorOffsetX=0.1555`
           `sensorOffsetY=-0.0010002`
           `cameraOffsetZ=-0.07187`
         - 当前已确认车体包络参数：
           `vehicleLength=0.70`
           `vehicleWidth=0.52`

      2) 修改 `src/readme.md`
         - 按现有版本记录风格补充本轮说明
         - 去掉顶部大段说明文写法，改回“版本号 + 改了什么”的形式
         - 单独记录当前最值得怀疑的参数 `useTerrainAnalysis`

      本轮已确认内容：
      - `local_planner` 运行时通过 `/way_point` 接收目标点，不是自己发布目标点
      - `sensorOffsetX`、`sensorOffsetY` 与 `car/urdf/car.urdf` 中 `lidar_joint` 的安装位姿一致
      - `vehicleLength=0.70`、`vehicleWidth=0.52` 与 `car/urdf/car.urdf` 中 `base_link` 网格包围盒尺寸基本一致
      - `local_planner` 内部将车体近似为以 `vehicle` 参考点为中心的矩形包络，不是从车头或车尾单边开始量

      当前链路：
      - `loam_interface` 输出 `/state_estimation` 和 `/registered_scan`
      - `terrain_analysis_ext` 输出 `/terrain_map`
      - `local_planner` 读取定位、目标点和环境信息，输出 `/path`
      - `pathFollower` 跟踪 `/path`，输出 `/cmd_vel`
      - `sensor_scan_generation` 与 `tare_planner` 继续负责探索链路

      当前重点怀疑参数：
      - `useTerrainAnalysis=true`
      - 该参数不是简单的“是否开地形增强”，而是直接决定 `local_planner` 使用哪一路环境输入
      - 当 `useTerrainAnalysis=true` 时，`local_planner` 主要使用 `/terrain_map`
      - 当 `useTerrainAnalysis=false` 时，代码理论上会退回 `/registered_scan` 分支，而不是设计上完全不能运行

      当前怀疑点：
      - 历史现象是“不打开地形检测时无法运行”
      - 但从代码逻辑看，`useTerrainAnalysis=false` 不应该直接让 `local_planner` 完全失效
      - 更可能的问题在于 `/registered_scan` 当前效果、`minRelZ/maxRelZ` 高度筛选，或者 `obstacleHeightThre` 等阈值与 scan 分支不匹配

      当前建议：
      - 先分别验证 `useTerrainAnalysis=true/false` 两种模式下 `/path` 是否能稳定输出
      - 如果 `false` 模式异常，优先联查 `/registered_scan`、`minRelZ`、`maxRelZ`、`obstacleHeightThre`
      - 现阶段优先把 `useTerrainAnalysis` 当成“规划输入模式切换开关”来看，而不是普通功能开关

26.4.7
   4.7.4
      版本目标：为 `robotcup_map` 新增一套独立的 TARE 探索入口，并针对“小尺度室内迷宫 + 窄通道入口容易直接判探索完成”的问题做专用调参记录。

      本次改动：
      1) 新增 `tare_planner/launch/explore_robocup.launch`
         - 固定加载 `robocup` 场景参数
         - 保留 `rviz`、`rosbag_record`、`use_boundary` 接口
         - 默认 bag 前缀为 `tare_robocup`

      2) 新增 `tare_planner/config/robocup.yaml`
         - 参数方向按 `robotcup_map` 当前形态单独收敛
         - 场景特征按“小尺度室内迷宫、窄通道、连续转角、入口较窄”处理
         - 相比 `indoor.yaml`，重点做了以下调整：
           `frontier` 保留条件放宽
           `viewpoint` 采样更密
           `lookahead` 与 waypoint 外延距离缩短
           `keypose graph` 连边距离缩短
           `grid world` 中 covered/exploring 切换门槛降低

      3) `robocup.yaml` 第二轮继续收紧参数
         - `kFrontierClusterMinSize` 继续下调
         - `kMinAddPointNumSmall`、`kMinAddPointNumBig`、`kMinAddFrontierPointNum` 继续下调
         - `kViewPointCollisionMargin` 继续减小
         - `viewpoint_manager` 分辨率继续加密
         - `kLookAheadDistance`、`kExtendWayPointDistanceBig/Small` 继续缩短
         - 目的不是让车更激进，而是避免在窄道入口附近过早把可探索区域判成 covered

      当前地图判断：
      - `~/.gazebo/models/robotcup_map/model.sdf` 中墙体基本由 `1.25 x 0.15 x 2.5` 的小段拼接组成
      - 该地图不适合直接沿用 `garage` 一类偏开阔场景的参数
      - 当前更接近“窄口连接的小尺度室内探索”而不是“大空间覆盖”

      当前验证现象：
      - 使用 `explore_robocup.launch` 后，原先“一启动或一到入口就立刻结束探索”的问题已有所缓解
      - 但当前仍会在部分窄道入口提示“探索完成”
      - 说明问题已从“完全不适配”缩小到“入口附近 frontier / candidate viewpoint 仍不足或仍被筛掉”

      当前建议：
      - 当前专用入口统一使用：
        `roslaunch tare_planner explore_robocup.launch`
      - 后续排查优先观察：
        `filtered_frontier_cloud`
        candidate viewpoints
        exploring cell marker
      - 若后续仍在入口结束，优先判断是：
        1) frontier 本身没有留下
        2) viewpoint 因碰撞/连通性被筛空
        3) cell 在入口被过早切到 covered

26.4.8
   4.8.0
      针对当前实车尺寸与路径跟踪参数继续调整 local_planner
      修改了 local_planner.launch：
      vehicleLength：0.70 -> 0.73
      vehicleWidth：0.52 -> 0.56
      lookAheadDis：1.5 -> 0.6

      说明：
      - 将车体长宽调整为更接近当前实车尺寸
      - 将前视距离缩短，尝试让路径跟踪更贴近局部路径

      当前发现的问题：
      - 车辆存在转向过度问题
      - 该问题在修改前存在，修改后仍然存在
      - 目前判断转向过度并非仅由车体尺寸或 lookAheadDis 引起，后续还需要结合 yawRateGain、stopYawRateGain、maxYawRate 以及车速相关参数继续排查

26.4.9
   4.9.0
      针对 local_planner 过于贴墙的问题继续调参
      修改了 local_planner.launch：
      vehicleLength：0.73 -> 0.72
      vehicleWidth：0.56 -> 0.54
      lookAheadDis：0.6 -> 0.3
      yawRateGain：7.5 -> 8.5
      stopYawRateGain：7.5 -> 8.5
      maxYawRate：90.0 -> 60.0

      说明：
      - 适当减小车体包络尺寸，避免局部规划中过早贴近障碍边界
      - 通过减小前视距离提高路径跟踪动态性，以适应更复杂的环境和更急的局部转向
      - 通过增大角速度增益抑制转向响应偏慢导致的过度转向问题
      - 同时降低最大角速度上限，避免控制输出过猛

      当前效果：
      - 主要改善了 local_planner 过于贴墙的问题

      当前结论：
      - `lookAheadDis` 不能继续减小到 `0.2`
      - 当 `lookAheadDis=0.2` 时，车辆不会运动

   4.9.1
      继续微调 local_planner 的车体包络尺寸，并补充 robocup 参数文件中文注释
      修改了 local_planner.launch：
      vehicleLength：0.72 -> 0.70
      vehicleWidth：0.54 -> 0.53

      修改了 robocup.yaml：
      - 为 robocup 场景参数补充了中文注释
      - 对话题接口、行为开关、前沿探索、滚动栅格、关键位姿图、视点管理、Grid World、可视化等部分增加了中文说明

      说明：
      - 本次尺寸调整继续朝更贴近实际车体包络的方向收敛
      - `robocup.yaml` 本次以可读性整理为主，便于后续针对窄通道、入口、探索完成判定等问题继续调参

   4.9.2
      针对 `robotcup_map` 中“宽路进入窄入口时路线犹豫、双入口前反复试探”的问题，先做第一轮 TARE 参数收敛

      修改了 robocup.yaml：
      - `kLookAheadDistance`：`2.5 -> 3.0`
      - `kUseMomentum`：`false -> true`
      - `kDirectionChangeCounterThr`：`6 -> 2`
      - `kDirectionNoChangeCounterThr`：`5 -> 8`
      - `kGreedyViewPointSampleRange`：`5 -> 1`
      - `kLocalPathOptimizationItrMax`：`12 -> 4`
      - `kViewPointCollisionMargin`：`0.18 -> 0.22`
      - `kCollisionFrameCountMax`：`2 -> 3`

      说明：
      - 这一轮不再继续动 `local_planner`，而是把问题定位到 TARE 的 `/way_point` 决策层
      - 调参目标不是让车更激进，而是减少双入口前随机换边、降低局部视点重选带来的来回犹豫
      - 其中：
        `kGreedyViewPointSampleRange` 与 `kLocalPathOptimizationItrMax` 主要用于降低局部视点选择的随机性
        `kUseMomentum` 与方向切换阈值主要用于抑制反复改主意
        `kViewPointCollisionMargin` 与 `kCollisionFrameCountMax` 主要用于让入口附近候选视点稳定一些

      当前结果：
      - 相比上一轮，入口前的犹豫和左右试探有所收敛
      - 但当前仍会出现掉头返回的情况

      当前问题判断：
      - 现阶段问题已不再主要表现为“进不去窄入口”
      - 更像是 TARE 在局部探索过程中仍会把“回头/换边”视为可接受动作
      - 对当前这台车来说，迷宫内部掉头空间很差，实际代价明显高于算法当前默认假设

      当前结论：
      - 下一轮不建议继续一味收紧 frontier、viewpoint 或碰撞门槛
      - 若继续沿这个方向收紧，风险是重新退回“可以减少犹豫，但又进不去迷宫窄道”
      - 后续调参更适合沿“增强已选方向延续性、减少回头收益”的方向继续做，而不是继续压缩入口可选空间

   4.9.3
      针对 `robotcup_map` 中丁字路口仍有左右犹豫、选边后不够果断的问题，继续沿 TARE `/way_point` 决策层收敛参数

      修改了 robocup.yaml：
      - `kExtendWayPointDistanceBig`：`2.0 -> 2.5`
      - `kLookAheadDistance`：`3.0 -> 3.5`

      说明：
      - 这一轮仍不继续动 `local_planner`，先增强 TARE 在局部分支选择后的方向延续性
      - `kExtendWayPointDistanceBig` 上调后，发布给下游的 `/way_point` 会更愿意落到已选分支更深处
      - `kLookAheadDistance` 上调后，局部路径上的前瞻点会更靠前，减少在丁字路口入口附近反复重选左右分支
      - 这次调参目标不是压缩可选空间，而是让“已经选了一边”之后更不容易马上改主意

      观察重点：
      - 如果丁字路口左右试探明显减少，说明问题主要还是 TARE 的局部分支延续性不足
      - 如果 `/way_point` 已经稳定落进某一侧，但车体仍在入口前摇头或切不进去，下一轮就该回到 `local_planner` 继续调跟踪参数

      当前效果：
      - 当前回环区域已经可以完整探索，说明这一轮对已选方向延续性的增强是有效的
      - 但在完成当前回环后，车辆还无法继续切换到回环外侧的另一个回环区域

      当前问题判断：
      - 当前问题已不再主要表现为丁字路口局部左右犹豫
      - 更像是 TARE 在完成一个闭环区域后，对外侧另一处可探索回环的切换与延伸能力仍然不足

      当前结论：
      - 下一轮不应继续只盯着丁字路口局部延续性
      - 更适合优先排查全局层面的 frontier 选择、global path 延伸，或回环外目标区域的可达性判定

26.4.16
   4.16.0
      版本目标：整理当前控制与感知相关改动，补充达妙底盘控制入口，并统一记录本轮包级调整内容。

      本次改动：
      1) 新增 `control` 包下的达妙差速底盘控制节点
         - 新增 `damiao_diff_chassis_node`
         - 支持速度模式与 MIT 模式
         - 通过 `control_mode_config.h` 切换控制模式
         - 将底盘公共参数、速度模式参数、MIT 模式参数分别拆到独立配置头文件
         - 启动时按官方文档先设置模式，再使能，再订阅 `/cmd_vel` 发送左右轮电机速度
         - 增加寄存器写入回执校验和使能状态反馈确认

      2) 在 `control` 包中补充达妙官方协议文档
         - 新增 `DM-J10010L-2EC减速电机说明书V1.1.pdf`
         - 新增 `调试助手使用说明书（达妙驱动控制协议）V1.4.pdf`

      3) 调整 `visual_obstacle_detector`
         - 删除旧的 `visual_detector.py`
         - 新增 `person_global_localizer.py`
         - 新增对应 launch 文件与模型文件入口
         - 补充 `tf`、`marker`、`message_filters` 等依赖声明

      4) 清理 `imu_run` 包
         - 删除原有云台/电机控制节点、键盘控制脚本和 launch

      5) 其他同步修改
         - `rplidar_a3.launch` 串口改为 `/dev/ttyUSB1`
         - `robocup.yaml` 继续调整 `kExtendWayPointDistanceBig` 与 `kLookAheadDistance`

      当前说明：
      - `control` 当前默认保留速度模式入口，如需切到 MIT 模式，修改 `control_mode_config.h` 即可
      - 达妙使能/失能、速度帧和 MIT 控制帧均已按本轮核对过的官方协议实现
   4.16.1
      继续调整local_planner，目标是让局部规划在窄通道和直角弯里更灵活，减少完全卡死
      修改了local_planner.launch：
      dirToVehicle：false -> true
      minPathRange：1.0 -> 0.3
      pathRangeStep：0.5 -> 0.1
      说明：
      1) dirToVehicle改为true后，局部候选路径筛选更贴近车体当前朝向，整体响应会更动态
      2) minPathRange减小后，局部规划在狭窄位置可以退到更短的动作，不会一上来就因为路径长度不够而卡住
      3) pathRangeStep减小后，搜索失败时会更细粒度地缩短路径，提升直角弯和死胡同口附近的可尝试空间
      当前效果：
      1) local规划表现比之前更动态
      2) 在直角弯道处完全卡死的情况有所减少
26.4.19
   4.19.0
      版本目标：排查达妙底盘 CAN 回执不匹配问题，修正反馈 ID 配置，并补充后续调试所需日志。

      本次改动：
      1) 调整 `control` 包下达妙底盘反馈 ID 配置
         - `chassis_common_config.h` 中 `kMasterId`：`0x000 -> 0x006`
         - 原因：实测抓包显示，寄存器写入回执和状态反馈均从 `0x006` 返回，而不是默认 `0x000`

      2) 补充 `damiao_diff_chassis_node` 调试日志
         - 启动日志增加 `feedback_id` 与 `register_frame_id`
         - 等待寄存器写入回执超时时，打印最后收到的一帧 CAN 报文
         - 等待状态反馈超时时，打印最后收到的一帧 CAN 报文

      抓包结论：
      - 写模式帧：`0x7FF [8] 05 00 55 0A 03 00 00 00`
      - 回执帧：`0x006 [8] 05 00 55 0A 03 00 00 00`
      - 当前速度模式控制帧仍为 `0x200 + ID`，电机 `5/7` 对应 `0x205/0x207`

      当前说明：
      - 达妙 CAN 固定波特率为 `1Mbps`
      - 本轮 `Network is down` 问题属于系统侧 `can0` 未正常 up，不是节点发帧逻辑本身错误
      - 当前反馈帧里电机 ID 仍按低 4 位校验，电机 `5/7` 可正常使用；若后续电机 ID 改到大于 `15`，需重新核对反馈匹配规则
   4.19.1
      版本目标：继续整理达妙底盘反馈 ID 匹配逻辑，把“固定反馈 ID”改为“按电机 ID 偏移计算反馈 ID”，避免把控制帧 ID 与反馈帧 ID 混为一类。

      本次改动：
      1) 调整 `control` 包下反馈 ID 配置方式
         - `chassis_common_config.h` 中由固定 `kMasterId` 改为 `kFeedbackIdOffset`
         - 当前按 `expected_feedback_id = motor_id + kFeedbackIdOffset` 匹配回执与状态反馈
         - 现阶段偏移量配置为 `0x001`

      2) 调整 `damiao_diff_chassis_node` 中等待回包的实现
         - 模式写入回执等待改为按当前电机 ID 计算预期反馈帧 ID
         - MIT 模式下 `PMAX/VMAX/TMAX` 写入回执等待也同步改为按当前电机 ID 计算
         - 使能状态反馈等待同样改为按当前电机 ID 计算

      3) 同步更新启动日志与文档说明
         - 启动日志中的 `feedback_id` 改为 `feedback_id_offset`
         - 明确当前逻辑不是按“发送帧 ID + 1”等待回包
         - 实际采用的是“电机 CAN ID + 1”作为反馈/回执帧 ID

      当前说明：
      - 当前寄存器写入帧仍使用 `0x7FF`
      - 速度模式控制帧仍使用 `0x200 + ID`
      - 当前抓包已确认电机 `5 -> 0x006`，按同样规则右电机 `7` 的预期反馈帧 ID 为 `0x008`
   4.19.2
      版本目标：修复达妙底盘控制节点中文日志显示为 `?` 的问题，保证终端调试输出可直接阅读。

      本次改动：
      1) 调整 `damiao_diff_chassis_node` 进程入口的 locale 初始化
         - 在 `main()` 中增加 `std::setlocale(LC_ALL, "")`
         - 增加 `std::locale::global(std::locale(""))`
         - 让 ROS/log4cxx 按系统 UTF-8 locale 输出日志

      当前效果：
      - 启动日志中的“启动达妙差速底盘控制节点”“速度模式”“未收到电机”等中文信息已可正常显示
      - 不再出现中文统一被替换为 `?` 的情况

      当前说明：
      - 本轮修改只影响日志输出编码，不改变 CAN 协议、控制逻辑和底盘参数
26.4.20
   4.20.0
      版本目标：统一关闭达妙底盘控制节点中的 timeout 配置，并明确 `0` 的实际语义，避免把 `0` 当成立刻超时。

      本次改动：
      1) 调整 `control` 包下达妙底盘公共超时配置
         - `chassis_common_config.h` 中 `kCmdTimeoutSec`：`0.3 -> 0.0`
         - `chassis_common_config.h` 中 `kRegisterAckTimeoutMs`：`100 -> 0`
         - `chassis_common_config.h` 中 `kStatusFeedbackTimeoutMs`：`100 -> 0`

      2) 调整 `damiao_diff_chassis_node` 对 timeout=0 的处理语义
         - 寄存器写入回执等待中，`timeout_ms <= 0` 改为一直等待，不再视为超时
         - 电机状态反馈等待中，`timeout_ms <= 0` 改为一直等待，不再视为超时
         - `cmd_vel` 超时停车逻辑中，只有 `cmd_timeout_sec > 0` 时才启用超时判定

      当前效果：
      - 达妙底盘节点当前已统一为“timeout=0 表示关闭超时限制”
      - 不会再因为把超时参数改成 `0` 导致启动阶段立刻判超时
      - 运行阶段不会再因为超过 `0.3s` 未收到新的 `cmd_vel` 自动置零停车

      当前说明：
      - 启动后如果 CAN 总线一直没有目标回执或状态反馈，节点现在会持续等待，需人工介入排查
      - 本轮未做远端推送，仅保留本地版本记录与代码变更
   4.20.1
      版本目标：恢复达妙底盘节点中“上位机等待回包”的超时为非 0，保留 `cmd_vel` 自动停车超时关闭状态。

      本次改动：
      1) 调整 `control` 包下达妙底盘回包等待超时配置
         - `chassis_common_config.h` 中 `kRegisterAckTimeoutMs`：`0 -> 100`
         - `chassis_common_config.h` 中 `kStatusFeedbackTimeoutMs`：`0 -> 100`

      2) 当前保留不变的配置
         - `chassis_common_config.h` 中 `kCmdTimeoutSec` 继续保持 `0.0`
         - 本轮没有新增“写入电机寄存器的 timeout 参数”这类寄存器写入项

      当前效果：
      - 模式写入、MIT 参数写入和使能状态确认重新回到“有限时间等待回包”的行为
      - 运行阶段仍不会因为 `cmd_vel` 长时间未更新而自动停车
26.4.21
   4.21.0
      版本目标：补齐 robocup 仿真下“已配准点云 -> 二维 LaserScan -> hector 建图”的链路，并避免 hector 额外发布 TF 干扰当前坐标关系。

      本次改动：
      1) 新增 `pointcloud_to_laserscan/launch/registered_scan_to_scan.launch`
         - 默认订阅 `vehicle_cropbox_filter` 输出的 `/cloud_registered_ego_filtered`
         - 输出二维激光话题 `/scan`
         - 默认目标坐标系设为 `lidar`
         - 高度切片范围设为 `-0.10 ~ 0.30`
         - 量程范围设为 `0.3 ~ 30.0`

      2) 调整 `sim_hector/launch/hector.launch`
         - `scan` remap：`/scan_filtered -> /scan`
         - `pub_map_odom_transform`：`true -> false`

      当前说明：
      - 当前 hector 直接使用 `registered_scan_to_scan.launch` 输出的 `/scan` 做二维建图
      - 关闭 `pub_map_odom_transform` 后，hector 不再额外发布 `map -> odom`，避免和当前链路里的 TF 关系打架
      - 当前 `hector.launch` 里 `odom_frame` 仍保持 `base_link`，本轮主要是统一 scan 输入与 TF 发布行为
26.4.24
   4.24.0
      版本目标：整理 `rebo24` 机器人模型包，补充当前仿真入口与使用说明，并将本轮版本推送到远端。

      本次改动：
      1) 新增 `rebo24` 机器人描述包并替换原 `car` 包
         - 当前车模目录由 `src/car` 切换为 `src/rebo24`
         - 保留四轮底盘基础结构，并同步导入新的 `meshes`、`urdf`、`launch` 与 `config`
         - 新增 `imu.STL` 以及相机/IMU 相关坐标系链

      2) 补齐 `rebo24/urdf/rebo24.urdf` 中的传感器与 Gazebo 配置
         - LiDAR 通过 `libgazebo_ros_velodyne_laser.so` 发布 `/velodyne_points`
         - 相机链包含 depth、infra1、infra2、color 与对应 optical frame
         - IMU 传感器挂在 `camera_imu_frame`，当前更新频率配置为 `200Hz`

      3) 新增 `rebo24` 的基础启动入口
         - `gazebo.launch`：启动 Gazebo 空场景、静态 TF 并生成 `rebo24` 模型
         - `display.launch`：发布 `robot_description`，启动 `joint_state_publisher_gui` 与 `robot_state_publisher`
         - `simbase.launch`：统一组合 Gazebo 与显示链路
         - `keyboard.launch`：提供 `teleop_twist_keyboard` 键盘控制入口

      4) 补充仓库文档
         - 根目录 `README.md` 新增 `rebo24` 包结构说明、依赖项与常用启动命令

      当前说明：
      - `gazebo.launch` 目前默认加载 `gazebo_ros/launch/empty_world.launch`
      - 仓库中的 `src/rebo24/worlds/1.world` 暂未接入默认启动链
      - `display.launch` 中的 RViz 节点当前仍保持注释状态
   4.24.1
      版本目标：根据实车测量结果更新 `control` 底盘物理参数，并同步记录到版本说明。

      本次改动：
      1) 修改 `src/control/include/control/chassis_common_config.h`
         - `kWheelTrackMeters`：`0.36 -> 0.2285`
         - `kWheelbaseMeters`：`0.40 -> 0.5552`
         - `kWheelRadiusMeters`：`0.075 -> 0.0680`

      当前说明：
      - 本轮参数基于实测值：左右同轴两轮距离 `228.5 mm`，前后轴间距离 `555.2 mm`，轮半径 `68.0 mm`
      - 以上参数用于差速控制中的底盘几何配置，以及线速度到轮速的换算
26.4.25
   4.25.0
      版本目标：继续整理 `rebo24` 仿真配置，补齐说明文档，并统一当前 Gazebo 与 FAST_LIO 的几何对齐约定。

      本次改动：
      1) 修正 `rebo24/urdf/rebo24.urdf` 中差速插件参数
         - `wheelSeparation`：`0.438 -> 0.466000391758278`
         - `wheelDiameter`：`0.058 -> 0.050`
         - 含义：按当前 `rebo24` 四个轮子 joint 的实际安装位置与轮子 mesh 外径重新对齐

      2) 调整 `fast_lio/config/velodyne.yaml` 中 Gazebo 外参
         - `mapping/extrinsic_T` 改为 `[-0.08376, 0.005038918391, 0.051879206396]`
         - 当前外参定义按 Gazebo 实际发布 IMU 的挂载点 `camera_imu_frame` 计算
         - 当前保持 `imu_topic=/camera/imu` 不变

      3) 补齐 `rebo24` 启动入口并整理 launch 说明
         - `rebo24/launch/simbase.launch` 用于统一启动 Gazebo 与显示链
         - `rebo24/launch/keyboard.launch` 作为键盘控制入口保留
         - `rebo24/launch/display.launch` 新增 `delay_sec` 参数，便于组合启动时延迟加载显示节点

      4) 更新仓库说明文档
         - 根目录 `README.md` 补充 `rebo24` 仿真约定、FAST_LIO 外参说明与常用启动命令

      当前说明：
      - 本轮没有改动 `rebo24` 的 IMU link 关系；Gazebo IMU 插件仍挂在 `camera_imu_frame`
      - `imu_link` 当前只是 URDF 结构件，不单独发布 Gazebo IMU 数据
      - `display.launch` 当前会尝试加载 `rebo24/urdf.rviz`，若本地未提供该文件，需要后续补齐或注释 RViz 节点
   4.25.1
      版本目标：把 RoboCup 探索链路与 `robocup.yaml` 的使用说明补记到 `src/readme.md`，统一放在版本记录里，避免说明分散到根目录 `README.md`。

      本次补充：
      1) 记录 RoboCup 推荐启动顺序
         - `roslaunch rebo24 simbase.launch`
         - `roslaunch fast_lio mapping_velodyne.launch`
         - `roslaunch loam_interface loam_interface.launch`
         - `roslaunch sensor_scan_generation sensor_scan_generation.launch`
         - `roslaunch terrain_analysis terrain_analysis.launch`
         - `roslaunch terrain_analysis_ext terrain_analysis_ext.launch`
         - `roslaunch local_planner local_planner.launch`
         - `roslaunch tare_planner explore_robocup.launch`

      2) 明确 `explore_robocup.launch` 的参数加载关系
         - 该入口会通过 `tare_planner/launch/explore.launch` 固定传入 `scenario=robocup`
         - 实际加载参数文件为 `tare_planner/config/robocup.yaml`
         - 可选接口保留 `rviz`、`rosbag_record`、`bag_path`、`bag_name_prefix` 与 `use_boundary`

      3) 记录当前关键话题链路
         - `loam_interface` 输出 `/registered_scan`
         - `sensor_scan_generation` 输出 `/state_estimation_at_scan`
         - `terrain_analysis` 输出 `/terrain_map`
         - `terrain_analysis_ext` 输出 `/terrain_map_ext`
         - `tare_planner` 输出 `/way_point`
         - `local_planner` 当前订阅 `/way_point`、`/registered_scan`、`/terrain_map` 和 `/navigation_boundary`

      4) 记录 `robocup.yaml` 当前的调参方向
         - 放宽 frontier 保留条件，避免窄入口附近的小 frontier 被过早滤掉
         - 加密 `viewpoint_manager` 采样，并减小视点碰撞边界，尽量保留入口内外候选视点
         - 启用 `kUseMomentum`，降低方向切换阈值，减少双入口前来回试探
         - 上调 `kExtendWayPointDistanceBig` 与 `kLookAheadDistance`，增强已选分支后的延续性
         - 下调 `kMinAddPointNumSmall` 与 `kMinAddPointNumBig`，避免窄口前因“新增信息不足”直接结束探索

      当前说明：
      - 当前这套参数面向“小尺度室内迷宫、窄通道、连续转角、入口较窄”的 RoboCup 地图
      - 调参重点不是让车更激进，而是减少入口附近过早判定“探索完成”和路口左右犹豫
26.4.26
   4.26.0
      版本目标：继续整理 RoboCup 探索调参结论，并记录当前出现的位置零漂问题与怀疑方向，方便后续排查。

      本次改动：
      1) 调整 `tare_planner/config/robocup.yaml`
         - 保持窄道可进入的前提下，继续优化 RoboCup 场景探索参数
         - 当前探索完整度相比前一轮已有明显提升
         - 为避免沿墙追逐零碎 frontier，当前参数回到更稳的 frontier 保留阈值
         - `kCoverageDilationRadius` 小幅上调到 `0.18`
         - `kCellCoveredToExploringThr` 保持为 `3`，用于减少局部 frontier 波动带来的反复重开

      当前问题：
      - 当前开始出现位置零漂问题，表现为运行过程中位姿逐渐漂移

      当前怀疑方向：
      1) 撞墙导致零漂
         - 可能是车体在探索过程中发生碰撞后，引起定位状态异常或累计误差放大
         - 若该方向成立，原因可能与 `local_planner` 或 `explore` 的规划过度贴墙有关
         - 虽然当前探索完整度已经提高，但仍需继续关注窄道、拐角和沿墙补扫阶段是否存在过于贴墙的问题

      2) 无效点云导致零漂
         - 可能是输入给定位/建图链路的点云质量不稳定，导致匹配退化
         - 该问题可能与 `t` 变化有关，也可能和当前仿真环境本身有关
         - 如果后续再次出现“无有效点云”或点云内容明显异常，应优先回查仿真插件、时间戳和话题数据质量

      当前说明：
      - 现阶段还不能确定零漂根因，以上两条为当前主要怀疑方向
      - 后续建议优先结合撞墙时刻、点云有效率、`/registered_scan` 质量和状态估计输出一起对照排查

26.4.30
   4.30.0
      版本目标：在 `local_planner` 中把共享线速度上限统一降到 `0.5m/s`，并把与线速度直接耦合的固定距离/加速度阈值按比例同步收缩，避免只降速度不改局部规划尺度，导致控制与搜索范围不匹配。

      本次改动：
      1) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - `maxSpeed`：`1.0 -> 0.5`
         - `autonomySpeed`：`1.0 -> 0.5`

      2) 按 `0.5` 比例同步调整 `localPlanner` 侧固定范围参数
         - `adjacentRange`：`3.5 -> 1.75`
         - `minPathRange`：`0.3 -> 0.15`
         - `pathRangeStep`：`0.1 -> 0.05`
         - `goalClearRange`：`0.5 -> 0.25`

      3) 按 `0.5` 比例同步调整 `pathFollower` 侧固定跟踪参数
         - `lookAheadDis`：`0.3 -> 0.15`
         - `maxAccel`：`2.5 -> 1.25`
         - `stopDisThre`：`0.3 -> 0.15`
         - `slowDwnDisThre`：`0.85 -> 0.425`

      说明：
      - 本轮只缩放了和线速度直接耦合的“固定距离/加速度阈值”。
      - `pathScaleBySpeed` 与 `pathRangeBySpeed` 在代码里本来就会根据 `joySpeed = autonomySpeed / maxSpeed` 自动缩放，因此本轮不再重复改动，避免双重缩放。
      - `yawRateGain`、`stopYawRateGain`、`maxYawRate`、`dirDiffThre` 这类角度/转向增益参数本轮保持不变，原因是它们不适合简单按线速度做线性减半。

      当前建议：
      - 后续实车或仿真观察重点放在：
        1) 窄通道入口是否比之前更稳
        2) 直角弯处是否因前视距离过短出现频繁抖动
        3) `maxAccel=1.25` 是否仍偏激进
      - 如果后续发现减速和停车仍偏猛，优先再单独下调 `maxAccel`，不建议先去动角速度增益。
   4.30.1
      版本目标：针对 `4.30.0` 半速参数组下“更容易贴墙”的现象，先同步下调路径跟踪侧的转向增益，避免线速度已经减半但角速度响应仍保持原强度。

      本次改动：
      1) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - `yawRateGain`：`8.5 -> 4.25`
         - `stopYawRateGain`：`8.5 -> 4.25`

      说明：
      - 本轮只同步调整转向增益，不改 `maxYawRate`、速度比例逻辑和局部规划搜索范围参数。
      - 当前处理思路是先削弱“半速下仍然过强的转向响应”，优先观察贴墙和内切是否收敛。

      当前建议：
      - 下一轮观察重点是：
        1) 窄通道内侧墙贴靠是否减轻
        2) 直角弯入口是否仍然出现明显内切
        3) 是否出现转向响应过慢、拐不过去的新问题
26.5.1
   5.1.0
      版本目标：补记当前 `0.2m/s` 的 `local_planner` 低速参数组，并在保持低速上限不变的前提下，小幅回调上一轮过度保守的局部搜索和转向阈值，减少丁字路口前“挪过去”和犹豫。

      本次改动：
      1) 补记当前 `0.2m/s` 低速组基线
         - `maxSpeed`：`0.5 -> 0.2`
         - `autonomySpeed`：`0.5 -> 0.2`
         - `maxAccel`：`1.25 -> 0.5`
         - `stopDisThre`：`0.15 -> 0.06`
         - `slowDwnDisThre`：`0.425 -> 0.17`
         - `goalClearRange`：`0.25 -> 0.1`

      2) 继续修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - `adjacentRange`：`0.7 -> 1.2`
         - `minPathRange`：`0.06 -> 0.25`
         - `pathRangeStep`：`0.02 -> 0.05`
         - `lookAheadDis`：`0.06 -> 0.15`
         - `yawRateGain`：`1.7 -> 2.0`
         - `stopYawRateGain`：`1.7 -> 2.0`
         - `dirDiffThre`：`0.1 -> 0.15`

      说明：
      - `pathFollower` 里的“低速”不是 `0.2m/s`，而是代码按 `|vehicleSpeed| < 2 * maxAccel / 100` 动态判断。
      - 以当前 `maxAccel=0.5` 计算，低速判定阈值是 `0.01m/s`，只有落到这个量级时才会切到 `stopYawRateGain` 分支。
      - 当前巡航目标仍是 `0.2m/s`，所以 `stopYawRateGain` 主要影响临近停车、极慢速起步和路口反复收油时的转向响应。

      当前建议：
      - 下一轮优先观察：
        1) 丁字路口入口是否比前一轮更果断
        2) 窄通道中是否重新出现贴墙或左右摆头
        3) 若仍有明显犹豫，优先继续小幅调 `stopYawRateGain` 或 `dirDiffThre`，不建议先继续缩短前视距离
   5.1.1
      版本目标：针对当前地图里 3 个典型卡顿点继续修正局部规划与探索策略，重点解决丁字路口/宽阔地进入窄道时的犹豫原地转、中央直道卡住、进入丁字路口后卡住，并顺带补强一部分探索不完全的问题。

      本次改动：
      1) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - `adjacentRange`：`1.2 -> 1.4`
         - 新增 `pathSwitchScoreRatio=1.08`
         - 新增 `pathCrossBranchSwitchScoreRatio=1.15`
         - 含义：
           提前看见路口和窄道出口，并给局部规划加上“路径切换滞回”，减少在两个都能走的分支之间来回改主意。

      2) 修改 `autonomous_exploration_development_environment/src/local_planner/src/localPlanner.cpp`
         - `searchRadius`：`0.45 -> 0.36`
         - 新增逐条候选路径执行缓存 `execPaths`
         - 新增 `clearPathScore`，不再只按 group 选路，而是先稳住候选方向，再在该方向内挑实际更顺的单条路径
         - 新增 `lastSelectedCandidateID` 和记忆保持逻辑
         - 左右跨分支切换时使用更高的切换门槛
         - 含义：
           重点抑制丁字路口和宽转窄入口处“朝两条可通路中间原地转”的现象，同时减少在中央直道和进入丁字路口后的局部路径抖动与卡死。

      3) 修改 `autonomous_exploration_development_environment/src/local_planner/paths/path_generator.m`
         - `searchRadius`：`0.45 -> 0.36`

      4) 重生成 `autonomous_exploration_development_environment/src/local_planner/paths/correspondences.txt`
         - 使离线路径对应关系与新的 `searchRadius=0.36` 保持一致，避免运行时仍按旧邻域关系评分。

      5) 修改 `tare_planner/config/robocup.yaml`
         - 新增：
           `kLookAheadKeepMinDistance=0.8`
           `kLookAheadSwitchScoreMargin=0.12`
           `kReturnHomeCandidateCountThreshold=8`
         - 调整：
           `kFrontierClusterMinSize`：`4 -> 3`
           `kMinAddPointNumSmall`：`12 -> 10`
           `kMinAddPointNumBig`：`22 -> 18`
           `kMinAddFrontierPointNum`：`3 -> 2`
         - 含义：
           在旧 lookahead 仍然合理时优先延续当前分支，降低路口附近反复重选；同时保留更小的 frontier 和更弱的新信息区域，缓解探索后期“还有地方但被过早当作探索完成”的情况。

      6) 修改 `tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h/.cpp`
         - 接入上述新参数
         - 新增 lookahead 保持逻辑与切换 margin
         - 新增 `return_home_candidate_count_`，只有连续多轮都满足条件才正式进入回家状态
         - 在“疑似完成但尚未确认”阶段，先不覆盖当前探索路径
         - 含义：
           避免车辆刚到路口或弱 frontier 区域就被过早打断当前探索，减少进入丁字路口后突然停住或转入回家流程的误判。

      解决问题总结：
      1) 修正车辆在丁字路口、以及宽阔地进入窄道时容易犹豫并朝两个可通行路口中间原地旋转的问题。
      2) 修正车辆在中间直道区域容易卡住的问题。
      3) 修正车辆进入丁字路口后容易卡住的问题。
      4) 额外完善了一部分探索不完全的问题，减少小 frontier 和弱新增信息区域被过早忽略。

      说明：
      - 本轮不是单纯继续放宽某一个阈值，而是同时从“局部路径切换稳定性”和“全局探索完成判定”两侧一起收敛问题。
      - `correspondences.txt` 为按新 `searchRadius` 重生成后的结果，因此 diff 很大，属于预期变化。
26.5.3
   5.3.0
      版本目标：记录 RoboCup 当前“19 cm 斜坡地面 + 车辆起伏”场景下，二维 LaserScan、地形分析和局部规划链路的高度参数调整，并重点保留改动前基线值，避免后续再混淆 `lidar` 切片阈值和 `terrain_map` 高差阈值。

      本次改动：
      1) 修改 `pointcloud_to_laserscan/launch/registered_scan_to_scan.launch`
         - `min_height`：`-0.10 -> 0.02`
         - `max_height`：`0.30 -> 0.45`
         - `range_min`：`0.30 -> 0.10`
         - 说明：
           当前 `/scan` 仍工作在 `target_frame=lidar` 下，因此 `min_height/max_height` 是直接对 `lidar` 系点云做高度切片，不应按 `imu` 或 `base_link` 的离地高度直接代入。
           本轮最终把 `min_height` 收到接近 `0`，目的是尽量不让 19 cm 斜坡和车辆起伏时的地面卷入二维 LaserScan，同时保留近处真正立障。

      2) 修改 `pcl_ros/launch/vehicle_cropbox_filter.launch`
         - `min_z`：`-0.20 -> -0.22`
         - `max_z`：`0.45 -> 0.50`
         - 说明：
           这里只是略微放宽 FAST-LIO 输出点云在车体系下的保留高度范围，避免上坡、下坡和车体起伏时过早把有效点裁掉。

      3) 修改 `autonomous_exploration_development_environment/src/terrain_analysis/launch/terrain_analysis.launch`
         - `vehicleHeight`：`0.30 -> 0.50`
         - `minRelZ`：`-0.20 -> -0.55`
         - `maxRelZ`：`0.20 -> 0.40`
         - `disRatioZ`：`0.20 -> 0.22`
         - 说明：
           `terrain_analysis` 输出 `/terrain_map`，其点的 `intensity` 表示点到估计地面的离地高差。
           这里的参数职责是“保住斜坡与起伏场景下的有效地形点，并计算高差”，不是直接做二维障碍切片。

      4) 修改 `autonomous_exploration_development_environment/src/terrain_analysis_ext/launch/terrain_analysis_ext.launch`
         - `vehicleHeight`：`0.30 -> 0.50`
         - `lowerBoundZ`：`-0.20 -> -0.55`
         - `upperBoundZ`：`0.30 -> 0.40`
         - `disRatioZ`：`0.10 -> 0.18`
         - 说明：
           `terrain_analysis_ext` 主要给 TARE 提供 `/terrain_map_ext`，用于远处扩展地形。
           当前主局部规划仍直接读 `/terrain_map`，因此这一组参数的重点是与近场地形图保持风格一致，而不是单独决定局部避障结果。

      5) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - `useTerrainAnalysis`：保持 `true`
         - `minRelZ`：`-0.20 -> -0.55`
         - `maxRelZ`：`0.30 -> 0.40`
         - `obstacleHeightThre`：`0.15 -> 0.24`
         - `groundHeightThre`：`0.15 -> 0.22`
         - `costHeightThre`：`0.10 -> 0.24`
         - 说明：
           当前 `useTerrainAnalysis=true` 且 `useCost=false`。
           因此 `local_planner` 真正最关键的不是 `groundHeightThre`，而是 `obstacleHeightThre`。
           代码在读取 `/terrain_map` 时，会先过滤 `intensity > obstacleHeightThre` 的点进入规划障碍集合；也就是说，这个阈值本质上是“terrain 高差超过多少就直接算障碍”。
           本轮把它定在 `0.24`，是为了给 `19 cm` 斜坡留出少量建图/姿态误差余量，但又不放得像 `0.28` 那样过宽。

      当前最终高度参数：
      1) `registered_scan_to_scan.launch`
         - `min_height=0.02`
         - `max_height=0.45`
         - `range_min=0.10`
      2) `vehicle_cropbox_filter.launch`
         - `min_z=-0.22`
         - `max_z=0.50`
      3) `terrain_analysis.launch`
         - `vehicleHeight=0.50`
         - `minRelZ=-0.55`
         - `maxRelZ=0.40`
         - `disRatioZ=0.22`
      4) `terrain_analysis_ext.launch`
         - `vehicleHeight=0.50`
         - `lowerBoundZ=-0.55`
         - `upperBoundZ=0.40`
         - `disRatioZ=0.18`
      5) `local_planner.launch`
         - `useTerrainAnalysis=true`
         - `minRelZ=-0.55`
         - `maxRelZ=0.40`
         - `obstacleHeightThre=0.24`
         - `groundHeightThre=0.22`
         - `costHeightThre=0.24`

      当前重点说明：
      - 必须区分两类“高度参数”：
        1) `registered_scan_to_scan` 的 `min_height/max_height`
           这是 `lidar` 坐标系下的二维切片阈值。
        2) `terrain_analysis` / `local_planner` 的 `vehicleHeight`、`obstacleHeightThre`
           这是围绕“点到地面的高差”或“相对车体高度窗口”工作的阈值。
      - 不能把 `lidar` 相对 `imu/base_link` 的安装高度，直接当成 `/scan` 切片阈值来套用。
      - 当前 `rebo24` 中 `lidar_joint.z=0.3057`、`imu_joint.z=0.18395`，二者差值约 `0.12175m`；这能说明 LiDAR 相对 IMU 的安装高度差，但不能直接推出 `/scan` 的最佳 `min_height`。
      - 当前如果后续出现“斜坡又被当地障”或“真正小障碍漏掉”，优先回查：
        1) `/scan` 实际切片效果
        2) `/terrain_map` 中 19 cm 斜坡对应的 `intensity`
        3) `local_planner` 的 `obstacleHeightThre`
