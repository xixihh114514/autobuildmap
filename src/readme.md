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

   3.32.1
      解决了嵌套git的问题

   3.25.0
      更改了滤波方式，使用pclros的corpbox来过滤车体点云

   3.28.0
      当日初始版本
   3.28.1
      local中允许车辆倒车

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
