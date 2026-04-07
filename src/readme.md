# RobotCup 小尺度室内探索运行说明

## 当前推荐用途

这套配置面向平面室内/迷宫类地图，重点优化：

- 窄通道通过
- 90 度拐角切入
- 丁字路口减小前后/左右振荡
- 小尺度地图上的 TARE 前沿探索

本轮不改 C++ 源码，只改 launch、TARE 配置和运行文档。

## 启动前说明

- 工作目录：`/home/lzk/robotcup2026`
- 默认探索入口：`tare_planner/launch/explore_robotcup_indoor.launch`
- 必须保留 terrain 两个节点，不能只开其中一个
- `sim_terrain.launch` 默认已改为 `checkTerrainConn:=false`
- 当前默认不使用 boundary 文件，先用无边界模式跑通室内小地图

建议每个终端先执行：

```bash
cd /home/lzk/robotcup2026
source devel/setup.bash
```

## 手动启动顺序

### 终端 1：Gazebo 底盘

```bash
roslaunch car simbase.launch
```

### 终端 2：FAST_LIO

```bash
roslaunch fast_lio mapping_velodyne.launch
```

### 终端 3：车体点云裁剪

```bash
roslaunch pcl_ros vehicle_cropbox_filter.launch
```

### 终端 4：LOAM 接口桥接

```bash
roslaunch loam_interface loam_interface.launch
```

### 终端 5：地形分析

`sim_terrain.launch` 会依次启动 `terrain_analysis` 和 `terrain_analysis_ext`。

```bash
roslaunch terrain_analysis_ext sim_terrain.launch
```

如需显式确认连通性检查关闭，也可以写成：

```bash
roslaunch terrain_analysis_ext sim_terrain.launch checkTerrainConn:=false
```

### 终端 6：局部规划

```bash
roslaunch local_planner local_planner.launch
```

当前 `local_planner.launch` 已切到小地图 profile，当前先做第一步稳丁字路口，关键默认值为：

- `twoWayDrive=false`
- `autonomySpeed=0.7`
- `adjacentRange=3.2`
- `dirWeight=0.01`
- `dirThre=120.0`
- `pathCropByGoal=false`
- `pathRangeBySpeed=false`
- `minPathScale=0.6`
- `lookAheadDis=0.6`
- `dirDiffThre=0.2`
- `yawRateGain=6.0`
- `stopYawRateGain=6.0`
- `switchTimeThre=1.2`

如需临时覆盖，可直接在命令行传参，例如：

```bash
roslaunch local_planner local_planner.launch autonomySpeed:=0.6 dirThre:=150.0
```

### 终端 7：扫描同步

```bash
roslaunch sensor_scan_generation sensor_scan_generation.launch
```

### 终端 8：探索

```bash
roslaunch tare_planner explore_robotcup_indoor.launch
```

如果不想开 RViz：

```bash
roslaunch tare_planner explore_robotcup_indoor.launch rviz:=false
```

## 新增的小地图探索 Profile

### local_planner

本轮把常用调参项提升成了 launch arg，重点是：

- 默认关闭倒车，先压住丁字路口的前后切换
- 收紧方向约束，减少路口左右选路抖动
- 固定路径前向检测范围，不再随速度放大
- 放宽跟踪加速门槛，减少拐角前反复停走
- 关闭按目标点裁剪路径，避免 waypoint 在墙后时局部路径过早截断

### TARE

新增场景配置：

- `tare_planner/config/robotcup_indoor.yaml`

新增探索入口：

- `tare_planner/launch/explore_robotcup_indoor.launch`

这套配置相对 `indoor.yaml` 的主要变化是：

- 更短的 waypoint 外延距离
- 更小的 TARE 前瞻距离
- 更小的 Frontier 最小点数
- 更密的局部视点分辨率
- 更小的视点碰撞边距
- 更短的关键位姿连边距离
- 更小的传感器覆盖半径和膨胀半径

目标是让 TARE 在小地图里少给“跨墙远点”，更多生成贴近局部可达空间的 waypoint。

## 运行时检查

建议启动完成后确认这些话题在持续发布：

```bash
rostopic list | grep -E "state_estimation|registered_scan|terrain_map|state_estimation_at_scan|way_point|path|cmd_vel"
```

重点链路应为：

- `FAST_LIO -> /Odometry + /cloud_registered`
- `vehicle_cropbox_filter -> /cloud_registered_ego_filtered`
- `loam_interface -> /state_estimation + /registered_scan`
- `sensor_scan_generation -> /state_estimation_at_scan + /sensor_scan`
- `TARE -> /way_point`
- `local_planner -> /path`
- `pathFollower -> /cmd_vel`

如果探索异常，优先检查：

- `/way_point` 是否在持续刷新
- `/path` 是否长期为空
- `/cmd_vel` 是否持续为 0
- `/terrain_map` 和 `/terrain_map_ext` 是否正常发布

## 当前验收目标

- 直角弯前不连续顶墙超过 3 秒
- waypoint 不长期落在墙后不可达位置
- 局部规划允许通过一次或多次倒车完成姿态修正
- 同一拐角前后切换不连续超过 2 个往返周期
- 单次运行尽量覆盖完整张 `robotcup_map` 主走廊与分支

以下为原始历史记录，保留备查。

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
