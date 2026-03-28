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
      修改了滤波点云，增加了滤波后的点云的数量
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