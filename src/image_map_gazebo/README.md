# image_map_gazebo

适用环境：Ubuntu 20.04 + ROS Noetic + Gazebo 11。

地图约定：
- 1 个图片方格 = 1.0 m；
- 图片上方 = Gazebo +Y 方向；
- 图片右方 = Gazebo +X 方向；
- 黑色线段 = 墙体；
- “箱子” = 木箱障碍物；
- “斜坡/楼梯/操作台” = 对应实体模型；
- “——”和“|” = 低矮路障；
- “门口” = 绿色入口标记，无碰撞。

## 使用方法

```bash
cd ~/catkin_ws/src
cp -r /path/to/image_map_gazebo .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch image_map_gazebo image_map.launch
```

## 重新生成 world

```bash
cd ~/catkin_ws/src/image_map_gazebo
/usr/bin/python3 scripts/generate_image_map_world.py
```

要调整比例，修改 `scripts/generate_image_map_world.py` 里的 `CELL`。
