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

   3.23.1
      解决了嵌套git的问题

26.3.25
   3.25.0
      更改了滤波方式，使用pclros的corpbox来过滤车体点云

26.3.28
   3.28.0
      当日初始版本
   3.28.1
      local中允许车辆倒车

26.3.30
   3.30.0
      当日初始版本
   3.30.1
      local_planner先做诊断性调参，验证窄通道和90度弯卡住是否主要由方向约束过强导致
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

26.4.12
   4.12.0
      FAST_LIO 在使用 mapping_velodyne.launch 运行时，出现“启动正常、运行一段时间后崩溃”的问题。
      终端典型报错为：
      malloc(): mismatching next->prev_size (unsorted)
      [laserMapping-1] process has died, exit code -6

      当前现象记录：
      1) 不是启动即崩，而是运行一段时间后才出现。
      2) 崩溃前可能出现 “No point, skip this scan!”。
      3) 当前使用 fast_lio/config/velodyne.yaml 与 fast_lio/launch/mapping_velodyne.launch。
      4) 当前参数核对后，velodyne.yaml 中 preprocess/timestamp_unit=0 与 rs_to_velodyne 输出 time 的秒单位是匹配的，因此不能简单认为是 velodyne.yaml 写错导致。

      当前判断：
      1) 该报错更像堆内存被提前破坏后的延迟报错，不一定是 malloc 当场出错。
      2) 更可疑位置在 FAST_LIO 的 Velodyne 特征提取路径，而不是 velodyne.yaml 单独配置错误。
      3) src/fast_lio/src/preprocess.cpp 中 give_feature() 对异常 ring/坏帧的边界保护不足，怀疑存在越界访问风险。
      4) 当前 launch 中 feature_extract_enable=1，属于较敏感路径，异常帧更容易在该路径触发问题。
      5) velodyne.yaml 中 pcd_save_en=true 且 interval=-1 会造成长时间运行时内存持续累积，但更像长期内存压力问题，不像本次 unsorted 报错的直接根因。
      修改了local_planner.launch：
      checkRotObstacle：true -> false
      dirWeight：0.02 -> 0.005
      pathCropByGoal：true -> false
      备注：dirThre原始默认值为90.0，当前180.0为手动调整，不属于本次代改内容

26.4.14
   4.14.0
      当日初始版本

   4.14.1
      新增了视觉目标检测包，并补充了与之配套的相机深度对齐和 FAST_LIO 点云保存参数调整

      本次改动：
      1) 新增 `visual_obstacle_detector`
         - 新增 `person_global_localizer.launch`
         - 新增 `person_global_localizer.py`
         - 使用 YOLO 对彩色图像进行目标检测
         - 结合对齐后的深度图与相机内参，恢复目标在相机坐标系下的 3D 位置
         - 发布目标点云、Marker 和调试图像，便于后续接入避障或可视化

      2) 修改 `car/launch/sensor.launch`
         - Realsense 启动参数增加 `align_depth:=true`
         - 使视觉节点可以直接使用 `/camera/aligned_depth_to_color/image_raw`

      3) 修改 `fast_lio/config/velodyne.yaml`
         - `pcd_save/interval`：`-1 -> 0`

      说明：
      - 当前视觉节点默认输入为彩色图、对齐深度图和相机内参
      - 当前检测结果按脚本实现发布在相机坐标系下，后续如需接入全局避障或全局标记，还需要继续补全坐标变换链
      - 当前默认识别类别为 `victim`
      - `interval=0` 后，不再按固定帧数切分保存 PCD，退出时仍会统一输出累计点云

      当前用途：
      - 为后续视觉避障 / 人员目标定位提供基础输入
      - 便于在 RViz 中直接观察检测框、深度取样结果和目标位置

26.4.22
   4.22.0
      新增实车底盘控制包，并补充点云转 LaserScan 与 Hector SLAM 的 2D 建图链路，同时对 local_planner 做一轮更贴近车体的调参

      本次改动：
      1) 新增 `control`
         - 新增 `damiao_diff_chassis_node`
         - 通过 SocketCAN 下发差速底盘左右轮控制
         - 当前保留速度模式与 MIT 模式两套配置，默认使用速度模式
         - 当前底盘参数记录为：
           `can0`、左电机 ID `5`、右电机 ID `7`
           轮距 `0.36m`、轴距 `0.40m`、轮半径 `0.075m`

      2) 新增 `pointcloud_to_laserscan`
         - 补充标准点云转 LaserScan 功能包
         - 便于将 3D 点云链路接入 `/scan`，供 2D 建图或定位节点使用

      3) 新增 `hector_slam/hector_slam_launch/launch/hector.launch`
         - 新增 Hector SLAM 建图启动文件
         - 当前订阅 `/scan`
         - 当前 `use_sim_time=true`
         - 当前关闭 `pub_map_odom_transform`，开启 `pub_map_scanmatch_transform`

      4) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - `dirToVehicle`：`false -> true`
         - `minPathRange`：`1.0 -> 0.3`
         - `pathRangeStep`：`0.5 -> 0.1`

      调整目的：
      - 让局部规划更按车体朝向筛选可行方向
      - 在窄通道、贴障或掉头困难场景下，允许更短路径与更细步长逐步收缩搜索范围

      当前用途：
      - 为 `real_car_3D` 分支补齐实车底盘控制与 2D 建图验证链路
      - 便于后续联调 `cmd_vel -> 底盘控制 -> /scan -> hector_slam`

   4.22.1
      针对实车 2D 建图链路继续补充参数调整与运行记录，完善 Hector SLAM 与点云转 LaserScan 的默认配置

      本次改动：
      1) 修改 `hector_slam/hector_slam_launch/launch/hector.launch`
         - `/use_sim_time`：`true -> false`
         - 启用 `hector_trajectory_server`
         - 启用 `hector_geotiff_node`
         - `map_file_path`：`$(find your_package_name)/maps -> $(find hector_geotiff)/maps`

      2) 修改 `pointcloud_to_laserscan/launch/registered_scan_to_scan.launch`
         - `scan_time`：`0.1 -> 0.05`
         - `range_min`：`0.3 -> 0.1`

      调整目的：
      - 将 Hector SLAM 默认时间源切换到实车模式，避免继续沿用仿真时间
      - 让 `/scan` 能保留更近距离障碍物，并以更高频率参与 2D 建图
      - 让建图链路在默认启动时同时具备轨迹发布与 GeoTIFF 地图导出能力

      说明：
      - 当前 GeoTIFF 输出目录使用 `hector_geotiff` 自带 `maps` 目录，避免示例占位包名导致 launch 解析失败
      - 如需回到 Gazebo 仿真，应将 `/use_sim_time` 改回 `true`

   4.22.2
      将视觉检测得到的人物中心点接入 Hector GeoTIFF 导出链路，使人物中心点能够以 `map` 坐标系标注到导出的 2D 地图中

      本次改动：
      1) 修改 `visual_obstacle_detector/person_global_localizer.py`
         - 保留原有“检测框中心深度”作为人物中心点定义
         - 新增 `tf2` 变换，将人物中心点从相机坐标系变换到 `map`
         - 在继续发布 `~person_camera_cloud` 的同时，新增发布 `~person_map_cloud`
         - 当前 `~person_map_cloud` 默认话题解析后为：
           `/person_global_localizer/person_map_cloud`

      2) 修改 `visual_obstacle_detector/launch/person_global_localizer.launch`
         - 新增 `global_frame` 参数，默认值为 `map`
         - 新增 `transform_timeout` 参数，默认值为 `0.05`

      3) 修改 `hector_slam/hector_geotiff_plugins`
         - 新增 `PersonMapWriter` 插件
         - 订阅 `map` 坐标系下的人物中心点云
         - 在 GeoTIFF 导出时将人物中心点绘制为目标标记
         - 当前默认标签格式为 `P1`、`P2`、`P3`

      4) 修改 `hector_slam/hector_slam_launch/launch/hector.launch`
         - 为 `hector_geotiff_node` 启用：
           `hector_geotiff_plugins/TrajectoryMapWriter`
           `hector_geotiff_plugins/PersonMapWriter`
         - 默认人物点云输入话题设置为：
           `/person_global_localizer/person_map_cloud`
         - 默认人物标注超时时间设置为 `1.5s`

      使用方式：
      - 启动 Hector SLAM：
        `roslaunch hector_slam_launch hector.launch`
      - 启动视觉人物定位：
        `roslaunch visual_obstacle_detector person_global_localizer.launch`
      - 在人物已被检测并且 TF 链正常时，人物中心点会持续发布到：
        `/person_global_localizer/person_map_cloud`
      - 需要导出带人物标注的 GeoTIFF 时，发送：
        `rostopic pub /syscommand std_msgs/String "data: 'savegeotiff'" -1`

      说明：
      - 当前标注点使用的是“人物中心点”，不是脚点或地面投影点
      - GeoTIFF 中是否绘制人物点，取决于保存时刻之前最近一段时间内是否收到了人物中心点云
      - 当前插件不会把视觉原始点云整片画进地图，而是仅绘制人物中心点标注，更适合 2D 地图使用

   4.22.3
      调整 GeoTIFF 中人物中心点的标注方式，使不同位置的人物在导出地图时使用不同颜色，并按稳定顺序编号为 `p1`、`p2`、`p3`

      本次改动：
      1) 修改 `hector_slam/hector_geotiff_plugins/src/person_geotiff_plugin.cpp`
         - 导出 GeoTIFF 前先按地图坐标对人物中心点排序
         - 当前采用 `x` 优先、`y` 次之的顺序生成稳定编号
         - 为每个编号使用不同颜色的调色板，而不是统一颜色

      2) 修改 `hector_slam/hector_slam_launch/launch/hector.launch`
         - `PersonMapWriter/label_prefix`：`P -> p`
         - 新增 `PersonMapWriter/use_palette_colors=true`

      说明：
      - 当前 `p1/p2/p3` 的顺序由保存时刻的人物中心点地图坐标决定
      - 当画面中人物集合变化时，编号会根据当前所有中心点重新排序
      - 若人数超过调色板长度，颜色会循环复用

   4.22.4
      将 GeoTIFF 中的人物标注改为累计保留模式：一旦某处人物中心点被登记，后续地图持续保留该标记；新位置仅在距离已有标记超过 `1m` 时才新增新的 `pN`

      本次改动：
      1) 修改 `hector_slam/hector_geotiff_plugins/src/person_geotiff_plugin.cpp`
         - 不再按当前帧检测结果重绘整套人物标签
         - 改为维护插件运行期间的持久化人物标记列表
         - 新检测到的人物中心点只有在距所有已有标记超过 `1m` 时才会登记为新标记
         - 已登记标记保留原始位置、原始编号和原始颜色，不会因为后续检测顺序变化而消失或重排

      2) 修改 `hector_slam/hector_slam_launch/launch/hector.launch`
         - 删除 `PersonMapWriter/point_timeout`
         - 新增 `PersonMapWriter/min_separation_distance=1.0`

      当前规则：
      - 第一次登记的位置记为 `p1`
      - 后续若在别处第一次出现且距离所有已有标记都超过 `1m`，则依次记为 `p2`、`p3`、`p4`
      - 若新检测点落在任一已有标记 `1m` 范围内，则认为是已有标记附近观测，不重复新增
      - 已生成的标记会持续出现在后续 GeoTIFF 地图中，直到重启 `hector_geotiff_node`

   4.22.5
      新增二维码检测与 GeoTIFF 永久标记链路，使用 OpenCV 解码二维码内容，并将二维码以 `cn+内容` 的标签形式永久保留在地图中

      本次改动：
      1) 新增 `visual_obstacle_detector/qr_global_localizer.py`
         - 使用 OpenCV `QRCodeDetector` 识别二维码
         - 从二维码四角中心点取深度，恢复二维码中心在相机坐标系下的 3D 位置
         - 将二维码中心点通过 TF 变换到 `map`
         - 发布 `~qr_map_markers`，其中标签格式为：
           `cn` + 扫描出的二维码内容
         - 同时发布调试图像，便于查看二维码框、中心点和深度

      2) 新增 `visual_obstacle_detector/launch/qr_global_localizer.launch`
         - 默认输入仍为彩色图、对齐深度图和相机内参
         - 默认全局坐标系为 `map`

      3) 新增 `hector_slam/hector_geotiff_plugins/src/qrcode_geotiff_plugin.cpp`
         - 新增 `QRCodeMapWriter`
         - 订阅 `/qr_global_localizer/qr_map_markers`
         - 将二维码标记以永久方式保存在插件内部
         - 当前同样加入 `1m` 去重校验，避免已有标记附近重复登记

      4) 修改 `hector_slam/hector_slam_launch/launch/hector.launch`
         - 为 `hector_geotiff_node` 启用 `hector_geotiff_plugins/QRCodeMapWriter`

      当前规则：
      - 第一次在某处识别到二维码，会生成一个新的永久标记
      - 标签格式为 `cn` 加二维码内容，例如二维码内容为 `A12`，则标签为 `cnA12`
      - 若后续新二维码中心点落在任一已有二维码标记 `1m` 范围内，则不重复新增
      - 已登记二维码会持续出现在后续 GeoTIFF 地图中，直到重启 `hector_geotiff_node`

   4.22.6
      补充记录本轮二维码地图标记方案的已知问题与边界，避免后续联调时把“当前实现限制”误判为新 bug

      当前想提醒的问题：
      - 二维码永久标记与人物永久标记目前都只保存在 `hector_geotiff_node` 插件进程内存中，重启该节点后会重新开始编号与登记，还没有做到跨重启落盘恢复
      - `QRCodeMapWriter` 当前的 `1m` 去重规则只按地图位置判断，不区分二维码内容；如果两个不同内容的二维码放得非常近，当前实现会只保留先登记的那个
      - 二维码标签会直接使用 `cn+二维码原文` 画到 GeoTIFF 上；如果二维码内容太长、带很多空格，或者本身是较复杂字符串，地图上文字可能会比较挤
      - 二维码三维定位当前取的是二维码中心附近深度中值，不是专门做平面角点重建；如果中心区域深度缺失、二维码斜放过大，或者深度和彩色图对齐不稳定，会出现“能识别内容但不落点”或落点轻微偏移
      - 当前二维码地图标记依赖彩色图、对齐深度图和相机内参同时正常输入；如果只有彩色图能看到二维码但深度不可用，当前实现不会登记该二维码

26.4.23
   4.23.1
      记录当前仍未完成的主线任务，作为下一阶段联调清单

      当前待完成任务：
      1) 重新导入 `URDF`
         - 重新确认模型、传感器插件、坐标系挂接和实际运行链路一致

      2) 重新核算 `FAST_LIO` 的距离相关结果
         - 重点确认当前距离/尺度是否正确，避免后续建图和定位继续带着系统偏差

      3) 调整 `IMU` 的 `link`
         - 重新梳理 `IMU` 所在坐标系定义和整车 TF 语义是否一致

      4) 调整 `pointcloud_to_laserscan` 的 `link`
         - 重新确认 `pcd2scan` 所使用坐标系与当前雷达/车体链路匹配

      5) 继续做视觉节点性能优化，并确认标记结果是否正确
         - 包括运行效率、检测稳定性、地图标记位置正确性和持续标记行为确认

      6) 对整个工作空间做一次完整检查
         - 包括 launch、TF、传感器输入、建图、标记链路和相关包之间的联调检查

   4.23.2
      修正二维码 GeoTIFF 标记中的文字截断与显示不清问题，并同步优化地图中对象标记的文字显示效果

      本次改动：
      1) 修改 `visual_obstacle_detector/scripts/qr_global_localizer.py`
         - 为 OpenCV `QRCodeDetector` 启用 alignment markers 支持
         - 对解码出的二维码文本增加可打印字符清洗，去掉隐藏控制字符
         - 当二维码原始解码结果与清洗后结果不一致时，输出节流告警，便于定位 OpenCV 解码异常

      2) 修改 `hector_slam/hector_geotiff/src/geotiff_writer/geotiff_writer.cpp`
         - 将 GeoTIFF 标签字符串从 `txt.c_str()` 改为按完整 UTF-8 字节串转换，避免遇到隐藏字符时中途截断
         - 放大对象标记文字绘制区域，避免 `cnrobocup1qr` 这一类较长标签被裁掉
         - 将对象标记文字颜色改为深绿色，增强与黑色墙体和白色空白区域的区分度
         - 将对象标记文字字号调大，提升导出地图中的可读性

      结果：
      - 二维码标签在 GeoTIFF 中不再容易出现只显示前半段的情况
      - `p...` 与 `cn...` 这一类对象标记在地图中的文字更清晰、更容易识别

   4.23.3
      为人物与二维码全局定位节点加入连续帧确认机制，并为对应 launch 补充中文注释，降低单帧误检直接进入地图标记链路的概率

      本次改动：
      1) 修改 `visual_obstacle_detector/scripts/person_global_localizer.py`
         - 新增人物检测连续帧确认逻辑，默认同一目标需连续命中 `3` 帧后才真正发布
         - 采用检测框中心最近邻匹配方式，在相邻帧间按像素位置连续跟踪同一人物
         - 调试图像中增加 `wait/ok` 与确认帧数显示，便于现场观察确认状态

      2) 修改 `visual_obstacle_detector/scripts/qr_global_localizer.py`
         - 新增二维码连续帧确认逻辑，默认同一二维码需连续命中 `3` 帧后才真正发布
         - 按二维码内容与中心像素位置持续累计确认帧数
         - 调试图像中增加确认进度显示，便于区分“已识别但未确认”和“已确认发布”

      3) 修改 `visual_obstacle_detector/launch/person_global_localizer.launch`
         - 新增 `min_confirm_frames`
         - 新增 `confirm_pixel_tolerance`
         - 新增 `confirm_timeout`
         - 为人物定位 launch 补充中文注释，说明各输入话题与确认参数含义

      4) 修改 `visual_obstacle_detector/launch/qr_global_localizer.launch`
         - 新增 `min_confirm_frames`
         - 新增 `confirm_pixel_tolerance`
         - 新增 `confirm_timeout`
         - 为二维码定位 launch 补充中文注释，说明各输入话题与确认参数含义

      当前默认规则：
      - 人物与二维码都需要连续 `3` 帧命中后才会真正进入后续地图标记链路
      - 若目标在相邻帧间跳变过大或中间丢失超过设定时间，则重新计数
      - 以上确认参数都已提升到 launch，可根据实车画面抖动情况继续调节

   4.23.4
      将 `visual_obstacle_detector` 的人物与二维码识别主节点从 Python 迁移到 C++，优先解决实车运行时的推理与图像处理性能问题，同时保持原有地图标记链路基本不变

      本次改动：
      1) 新增 `visual_obstacle_detector/src/person_global_localizer_node.cpp`
         - 使用 C++ `OpenCV DNN` 加载 `models/yolo.onnx`
         - 保留彩色图、对齐深度图、相机内参同步输入方式
         - 保留人物中心点深度恢复、TF 变换到 `map`、相机系与地图系点云发布
         - 保留连续帧确认逻辑与调试图像输出

      2) 新增 `visual_obstacle_detector/src/qr_global_localizer_node.cpp`
         - 使用 C++ `cv::QRCodeDetector` 做二维码检测与解码
         - 保留二维码中心点深度恢复、TF 变换到 `map`、地图标记发布与调试图像输出
         - 保留连续帧确认逻辑

      3) 修改 `visual_obstacle_detector/launch/person_global_localizer.launch`
         - 默认节点切换为 `person_global_localizer_node`
         - 默认模型路径从 `yolo.pt` 调整为 `models/yolo.onnx`

      4) 修改 `visual_obstacle_detector/launch/qr_global_localizer.launch`
         - 默认节点切换为 `qr_global_localizer_node`

      5) 修改 `visual_obstacle_detector/CMakeLists.txt` 与 `package.xml`
         - 新增 `roscpp`、`roslib`、`libopencv-dev` 等 C++ 侧构建依赖
         - 新增两个 C++ 可执行文件的编译与安装规则

      当前需要注意：
      - 人物检测的对外话题职责和原先基本保持一致，但默认模型已切换到 `yolo.onnx`
      - 二维码节点当前是单目标版本，一帧内只处理一个二维码
      - 之所以暂时不能直接做多二维码，是因为当前工作空间链接的 C++ OpenCV 版本仍为 `4.2.0`，缺少新版多二维码相关接口

      验证情况：
      - `catkin_make --pkg visual_obstacle_detector` 已编译通过

   4.23.5
      为人物检测节点接入 Intel 核显 OpenCL 推理路径，并将二维码节点继续保留在 CPU 路径，避免二维码与 `YOLO ONNX` 在同一块核显上争抢算力

      本次改动：
      1) 修改 `visual_obstacle_detector/src/person_global_localizer_node.cpp`
         - 新增 `compute_target` 参数，支持 `auto / cpu / opencl / opencl_fp16`
         - 默认 `auto`，优先尝试 `OpenCL`，失败时自动回退到 `CPU`
         - 节点启动时会输出当前 OpenCL 设备信息，便于现场确认是否真的跑到核显

      2) 修改 `visual_obstacle_detector/launch/person_global_localizer.launch`
         - 新增本地 OpenCL 运行环境相关参数
         - 默认向人物节点注入：
           `OPENCL_VENDOR_PATH=$(env HOME)/.local/intel-opencl-runtime/vendors`
         - 默认向人物节点注入本地 Intel OpenCL 运行时库路径

      3) 本地运行时目录
         - 当前核显运行时已准备在：
           `~/.local/intel-opencl-runtime`
         - 这套目录来自用户态解包，不依赖 `sudo apt install`

      当前需要注意：
      - 人物节点的 `YOLO ONNX` 推理链路已经明确支持 `OpenCL/OpenCL FP16`
      - 二维码节点当前刻意不接入核显，原因是二维码这条链路没有像 DNN 那样明确的推理 target，而且收益不确定，反而可能和人物检测争抢同一块核显资源
      - 如果后续本地 OpenCL 运行时目录被删除，人物节点会退回普通 CPU 路径；二维码节点本身就继续保持 CPU 路径

      验证情况：
      - `catkin_make --pkg visual_obstacle_detector` 已编译通过
      - 本地 `clinfo` 已能识别 `Intel(R) OpenCL HD Graphics`
      - 系统 C++ OpenCV 4.2 已实测可见 `OpenCL 2.1 NEO` 设备

   4.23.6
      回滚人物默认启动链路到 Python 版本，原因是当前系统侧 C++ OpenCV `4.2.0` 仍无法稳定解析现有 `YOLOv8 ONNX` 模型，直接走 C++ 节点会在启动阶段退出

      当前结论：
      - 人物 `C++ + OpenCL` 代码仍然保留在仓库中，后续可以继续用于升级验证
      - 但默认 `person_global_localizer.launch` 已恢复为 Python 节点，模型默认重新使用 `yolo.pt`
      - 这样做的目的是先保证实车链路稳定可启动，再等待后续升级 C++ OpenCV 或改用其他推理后端

      已定位到的实际问题：
      - 当前 `yolo.onnx` 在系统 C++ OpenCV `4.2.0` 下会在 `readNetFromONNX()` 阶段报错
      - 这不是 TF、launch 写法或核显配置本身的问题，而是旧版 C++ OpenCV 对当前 ONNX 导出格式支持不完整

      当前默认启动方式：
      - `roslaunch visual_obstacle_detector person_global_localizer.launch`
      - 上述命令现在会回到和之前相同的 Python 路径

26.4.26
   4.26.0
      当日初始版本
   4.26.1
      版本目标：补入 `rebo24` 车体描述包，并同步当前实车控制、感知与探索默认参数到仓库主线。

      本次改动：
      1) 新增 `rebo24` 描述包
         - 新增 `urdf/rebo24.urdf`、`meshes/`、`config/joint_names_rebo24.yaml`
         - 新增 `launch/display.launch`、`gazebo.launch`、`simbase.launch`、`sensor.launch`、`keyboard.launch`
         - 当前已具备基础显示、Gazebo 拉起、键盘遥控与实车传感器启动入口

      2) 修改 `control/include/control/chassis_common_config.h`
         - `kWheelTrackMeters`：`0.36 -> 0.2285`
         - `kWheelbaseMeters`：`0.40 -> 0.5552`
         - `kWheelRadiusMeters`：`0.075 -> 0.0680`
         - 目的：同步当前底盘控制侧使用的几何参数

      3) 修改 `hipnuc_imu/config/hipnuc_config.yaml`
         - `frame_id`：`base_link -> imu_link`
         - 目的：让 IMU 消息坐标系回到独立 `imu_link`，便于和车体 TF 链对齐

      4) 修改 `fast_lio/launch/mapping_velodyne.launch`
         - `rviz`：`true -> false`
         - 目的：默认启动建图时不再额外拉起 RViz，减轻现场启动负担

      5) 修改 `tare_planner/config/robocup.yaml`
         - 上调 `kExtendWayPointDistanceBig`、`kLookAheadDistance`
         - 调整 `kCoverageDilationRadius`、`kCellCoveredToExploringThr`
         - 保持 `kFrontierClusterMinSize=4`、`kMinAddFrontierPointNum=3`
         - 目的：增强路口方向延续性，减少沿墙碎小 frontier 反复诱导贴墙补扫

      6) 修改 `visual_obstacle_detector/launch/person_global_localizer.launch`
         - 默认模型：`yolo.onnx -> yolo.pt`
         - 默认节点：`person_global_localizer_node -> person_global_localizer.py`
         - 移除 `compute_target` 与本地 OpenCL 环境注入参数
         - 目的：默认链路继续回到已验证可用的 Python 人物检测路径

      当前可用入口：
      - 仿真模型：`roslaunch rebo24 simbase.launch`
      - 实车传感器：`roslaunch rebo24 sensor.launch`
      - 人物检测：`roslaunch visual_obstacle_detector person_global_localizer.launch`

      当前说明：
      - `rebo24` 目前主要是车体描述与启动入口包，后续如切主车体，仍需继续核对控制参数、TF 和实传感器话题
      - 本轮默认值整体偏向“先保证稳定可启动”，尤其是人物检测默认链路与 FAST_LIO 的 RViz 启动行为已按现场使用回调
   4.26.2
      版本目标：将视觉检测统一到 C++ 路线，并优先使用 Intel 核显，减少 FAST_LIO / tare / hector_slam 对 CPU 的竞争。

      本次改动：
      1) person_global_localizer 从 Python/OpenCV DNN 路线切换为 C++ 节点
         - launch 默认节点改为 `person_global_localizer_node`
         - 模型默认改为 `models/yolo.onnx`
         - 推理后端改为 OpenVINO
         - 默认 `compute_target=gpu`

      2) person 检测新增 OpenVINO 独立后端库
         - 新增 `visual_obstacle_detector/include/visual_obstacle_detector/openvino_person_backend.h`
         - 新增 `visual_obstacle_detector/src/openvino_person_backend.cpp`
         - 由于 pip 版 OpenVINO 使用 `_GLIBCXX_USE_CXX11_ABI=0`，不能直接把整个 ROS 包切到同一 ABI
         - 当前做法为：ROS 节点保持原 ABI，OpenVINO 推理后端单独隔离，避免 roscpp/cv_bridge/tf2 链接冲突

      3) qr_global_localizer 保持 C++ + OpenCV `QRCodeDetector`
         - 未改成 OpenVINO
         - 新增 `compute_target` 参数，支持 `auto / gpu / cpu`
         - `gpu` 路线走 OpenCV OpenCL + UMat
         - launch 默认 `compute_target=gpu`

      4) launch 层统一
         - `person_global_localizer.launch` 与 `qr_global_localizer.launch`
         - 现在都支持：
           `compute_target:=auto|gpu|cpu`
         - 当前默认均为：
           `compute_target:=gpu`

      编译/启动验证结果：
      1) `person_global_localizer_node` 编译通过
      2) `qr_global_localizer_node` 编译通过
      3) person 启动日志确认可识别 Intel iGPU，并正常在 GPU 上加载 OpenVINO 模型
      4) qr 启动日志确认可识别 Intel OpenCL 设备，并正常启用 OpenCL 路线

      性能记录（本机 NUC8i5BEH）：
      1) person(OpenVINO GPU) 单次推理约 `14.8 ms`
      2) qr(CPU) 单帧约 `20.2 ms`
      3) qr(OpenCL GPU) 单帧约 `21.2 ms`
      4) 并发时：
         - person GPU + qr CPU：person 约 `14.86 ms`
         - person GPU + qr GPU：person 约 `15.03 ms`

      结论：
      1) person 放核显有明显收益，应优先使用 GPU
      2) qr 放核显不会更快，甚至略慢，但可以减少 CPU 占用
      3) 如果系统同时运行 FAST_LIO、tare_planner、hector_slam，这些模块主要吃 CPU，因此当前默认策略仍建议：
         - person：GPU
         - qr：GPU
         目的是优先给 FAST_LIO / tare / hector_slam 腾出 CPU

      当前默认启动方式：
      1) `roslaunch visual_obstacle_detector person_global_localizer.launch`
      2) `roslaunch visual_obstacle_detector qr_global_localizer.launch`
      3) 若需手动切回 CPU，可附加：
         `compute_target:=cpu`

26.4.27
   4.27.0
      版本目标：修复 `rebo24` 传感器启动链路，并将二维码全局定位默认切换到更稳定的 `C++ + ZXing-cpp` 路线，避免系统 OpenCV `QRCodeDetector` 因缺少 QUIRC 无法正常解码。

      本次改动：
      1) 修复 `rebo24/sensor.launch`
         - 不再依赖已删除的 `car` 包启动入口
         - 新增 `rebo24/scripts/delayed_roslaunch.sh`，统一转发 `roslaunch`，并过滤 ROS 自动注入参数
         - `sensor.launch` 现支持 `start_lidar`、`start_imu`、`start_camera` 三个独立开关
         - 增加分阶段延时启动，默认先拉激光，再拉 IMU，最后拉相机，降低现场同时抢占资源的风险
         - `rebo24/CMakeLists.txt` 已补上脚本安装规则，保证节点类型可被 ROS 正常找到

      2) `qr_global_localizer` 默认切换为 `C++ + ZXing-cpp`
         - `qr_global_localizer.launch` 默认 `backend:=cpp`
         - C++ 主节点继续保留 OpenCV 图像预处理、深度采样、TF 变换与调试绘制
         - 二维码解码后端改为独立 `ZXing-cpp`，本地 vendor 路径为 `vendor/zxing_cpp`
         - 当前二维码解码本体仍运行在 CPU；`compute_target:=gpu` 只作用于 OpenCV/OpenCL 预处理，不代表整条 QR 链路已经改成 GPU 解码
         - Python 路线仍保留，可在需要时手动切回 `backend:=python`

      3) 切换原因
         - 当前系统 C++ 链接的是 `OpenCV 4.2.0`
         - 实测会报错：
           `Library QUIRC is not linked. No decoding is performed.`
         - 因此不再继续依赖系统 `QRCodeDetector`，改为仓库内自带的稳定 C++ 解码后端

      4) 其他同步调整
         - `hector_slam_launch/launch/hector.launch` 中 `geotiff_save_period` 从 `1` 调整为 `5`
         - 目的是减少高频地图落盘对 CPU 与磁盘的额外占用

      编译与启动验证：
      1) `catkin_make --pkg visual_obstacle_detector`
      2) `roslaunch --nodes visual_obstacle_detector qr_global_localizer.launch`
      3) `ldd devel/lib/visual_obstacle_detector/qr_global_localizer_node`
         - 已确认运行时加载本地 `vendor/zxing_cpp` 下的 `libZXingCore.so`

      当前建议：
      - 人物检测继续优先走 `person_global_localizer.launch` 默认 C++ 路线
      - 二维码检测默认走 `qr_global_localizer.launch backend:=cpp`
      - 需要特别注意：当前 QR 属于“CPU 解码 + 可选 OpenCL 预处理”路线，暂时还不是纯 GPU 解码
      - 当系统同时运行 FAST_LIO、tare_planner、hector_slam 时，优先把视觉任务留在 GPU / OpenCL 或独立推理后端上，以尽量给建图与规划腾出 CPU

26.4.29
   4.29.0
      版本目标：把现场常用的 TARE 前置链路与视觉检测链路都整理成一键启动入口，减少手动分段拉起时漏步骤或顺序不一致的问题。

      本次改动：
      1) 新增 `autonomous_exploration_development_environment/src/sensor_scan_generation/launch/base_tare.launch`
         - 将原先分开启动的：
           `sim_terrain.launch -> local_planner.launch -> sensor_scan_generation.launch`
           整理为单一入口
         - 并将 `sim_terrain.launch` 内部原本包含的三个启动项直接展开为：
           `loam_interface.launch -> terrain_analysis.launch -> terrain_analysis_ext.launch`
         - 当前完整启动顺序调整为：
           `loam_interface -> terrain_analysis -> terrain_analysis_ext -> local_planner -> sensor_scan_generation`
         - 相邻阶段默认延迟 `5s`
         - 即默认启动时刻分别为：
           `0s / 5s / 10s / 15s / 20s`
         - `terrain_analysis_ext` 继续固定使用：
           `checkTerrainConn:=false`

      2) 修改 `autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
         - 新增 `delay_sec` 参数
         - `localPlanner`、`pathFollower` 与两个静态 TF 发布节点均支持按统一延迟启动
         - 目的是让 `base_tare.launch` 可以直接接管启动节奏，而不需要额外脚本包装

      3) 修改 `autonomous_exploration_development_environment/src/sensor_scan_generation/launch/sensor_scan_generation.launch`
         - 新增 `delay_sec` 参数
         - 支持被 `base_tare.launch` 延迟拉起

      4) 新增 `visual_obstacle_detector/launch/visual.launch`
         - 将 `person_global_localizer.launch` 与 `qr_global_localizer.launch` 合并为统一视觉入口
         - 共享彩色图、深度图、相机内参、`compute_target`、`global_frame` 等公共参数
         - 同时保留人物检测与二维码检测各自独立的确认帧数、像素容差、后端等专属参数

      5) 文档同步
         - `teach,md` 已同步改为新的默认启动入口，避免现场执行时继续沿用旧命令

      当前推荐启动方式：
      1) TARE 前置链路：
         `roslaunch sensor_scan_generation base_tare.launch`
      2) 视觉检测链路：
         `roslaunch visual_obstacle_detector visual.launch`

      验证情况：
      1) `xmllint --noout` 已检查：
         - `base_tare.launch`
         - `local_planner.launch`
         - `sensor_scan_generation.launch`
         - `visual.launch`
      2) `source devel/setup.bash && roslaunch --nodes sensor_scan_generation base_tare.launch`
         - 已成功展开：
           `/loamInterface /tf_map_to_camera_init_bridge /terrainAnalysis /terrainAnalysisExt /localPlanner /pathFollower /vehicleTransPublisher /sensorTransPublisher /sensorScanGeneration`
      3) `source devel/setup.bash && roslaunch --nodes visual_obstacle_detector visual.launch`
         - 已成功展开：
           `/person_global_localizer /qr_global_localizer`
