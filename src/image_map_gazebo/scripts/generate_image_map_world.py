#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
根据示意图生成 Gazebo Classic / ROS Noetic 可用的 .world 地图。
环境：Ubuntu 20.04 + ROS Noetic + Gazebo 11
约定：1 个方格 = 1.0 m；图片上方 = Gazebo +Y 方向；图片右方 = Gazebo +X 方向。
"""
import math
import os
from pathlib import Path

CELL = 1.0          # 每个网格边长，单位 m；要放大地图只改这里
WALL_H = 1.2        # 墙高 1.20 m
WALL_T = 0.08       # 墙厚
FLOOR_Z = 0.0
SECOND_FLOOR_H = 1.2  # 二层/楼梯/高斜坡统一高度设为 1.20 m
SMALL_RAMP_H = 0.10   # 左上角两个斜坡为小斜坡，高度为 0.10 m
CENTER_RIDGE_H = 0.10 # “——”和“|”表示双斜坡在中间拼接的最高线，高度为 0.10 m


WORLD_NAME = "image_map_world"

items = []


def material(rgba: str) -> str:
    return f"""
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
        </material>"""


def add_box(name, x, y, z, sx, sy, sz, rgba="0.7 0.7 0.7 1", yaw=0.0, roll=0.0, pitch=0.0, collision=True):
    """添加长方体模型。x/y/z 为模型中心坐标。"""
    collision_xml = ""
    if collision:
        collision_xml = f"""
      <collision name='{name}_collision'>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      </collision>"""

    items.append(f"""
  <model name='{name}'>
    <static>true</static>
    <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
    <link name='link'>
      {collision_xml}
      <visual name='{name}_visual'>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        {material(rgba)}
      </visual>
    </link>
  </model>""")


def add_cylinder(name, x, y, z, radius, length, rgba="0.8 0.8 0.8 1"):
    items.append(f"""
  <model name='{name}'>
    <static>true</static>
    <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
    <link name='link'>
      <collision name='{name}_collision'>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
      </collision>
      <visual name='{name}_visual'>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
        {material(rgba)}
      </visual>
    </link>
  </model>""")


def add_second_floor(name, c, r):
    """图中数字“2”的位置：表示二层平台，平台顶面高度为 SECOND_FLOOR_H。"""
    x, y = cell_center(c, r)
    add_box(name, x, y, SECOND_FLOOR_H / 2.0,
            0.96 * CELL, 0.96 * CELL, SECOND_FLOOR_H,
            "0.62 0.62 0.62 1")


def cell_center(c, r):
    """8x8 网格的单元中心。c 从左到右 0~7，r 从上到下 0~7。"""
    x = (c - 3.5) * CELL
    y = (3.5 - r) * CELL
    return x, y


def edge_point(c, r):
    """网格线交点。c/r 为边界索引 0~8。"""
    x = (c - 4.0) * CELL
    y = (4.0 - r) * CELL
    return x, y


def add_wall_h(name, c1, c2, r):
    """水平墙段：从第 c1 条竖线到第 c2 条竖线，位于第 r 条横线。"""
    x1, y = edge_point(c1, r)
    x2, _ = edge_point(c2, r)
    cx = (x1 + x2) / 2.0
    length = abs(x2 - x1)
    add_box(name, cx, y, WALL_H / 2.0, length, WALL_T, WALL_H, "0.05 0.05 0.05 1")


def add_wall_v(name, c, r1, r2):
    """竖直墙段：位于第 c 条竖线，从第 r1 条横线到第 r2 条横线。"""
    x, y1 = edge_point(c, r1)
    _, y2 = edge_point(c, r2)
    cy = (y1 + y2) / 2.0
    length = abs(y2 - y1)
    add_box(name, x, cy, WALL_H / 2.0, WALL_T, length, WALL_H, "0.05 0.05 0.05 1")


def add_grid_visual():
    """地面浅色网格，只作视觉参考，不参与碰撞。"""
    for c in range(9):
        x, _ = edge_point(c, 0)
        add_box(f"grid_v_{c}", x, 0, 0.006, 0.01, 8 * CELL, 0.01, "0.82 0.82 0.82 1", collision=False)
    for r in range(9):
        _, y = edge_point(0, r)
        add_box(f"grid_h_{r}", 0, y, 0.006, 8 * CELL, 0.01, 0.01, "0.82 0.82 0.82 1", collision=False)


def add_ramp_to_height(name, c, r, direction, target_h, length_scale=1.35, color="0.45 0.45 0.45 1"):
    """添加斜坡。target_h 为斜坡连接的目标高度（m）。"""
    x, y = cell_center(c, r)
    length = length_scale * CELL
    width = 0.72 * CELL
    thick = 0.08
    angle = math.atan2(target_h, length)

    yaw_map = {
        "right": 0.0,
        "up": math.pi / 2,
        "left": math.pi,
        "down": -math.pi / 2,
    }
    yaw = yaw_map[direction]
    # 让斜坡低端尽量贴近地面，同时保证不会明显下沉。
    z = (length * abs(math.sin(angle)) + thick * abs(math.cos(angle))) / 2.0 + 0.005
    add_box(name, x, y, z, length, width, thick, color, yaw=yaw, pitch=-angle)


def add_ramp(name, c, r, direction):
    """添加连接地面与二层平台的标准斜坡。"""
    add_ramp_to_height(name, c, r, direction, SECOND_FLOOR_H, length_scale=1.35)


def add_small_ramp(name, c, r, direction):
    """左上角两个小斜坡：高度为 0.10 m。"""
    add_ramp_to_height(name, c, r, direction, SMALL_RAMP_H, length_scale=0.85)


def add_stairs(name, c, r, direction="up"):
    """添加连接地面与二层平台的楼梯。"""
    x, y = cell_center(c, r)
    yaw_map = {
        "right": 0.0,
        "up": math.pi / 2,
        "left": math.pi,
        "down": -math.pi / 2,
    }
    yaw = yaw_map.get(direction, math.pi / 2)
    steps = 5
    step_len = 0.16 * CELL
    step_w = 0.72 * CELL
    for i in range(steps):
        local_x = -0.36 * CELL + i * 0.18 * CELL
        wx = x + local_x * math.cos(yaw)
        wy = y + local_x * math.sin(yaw)
        h = SECOND_FLOOR_H * (i + 1) / steps
        add_box(f"{name}_step_{i+1}", wx, wy, h / 2.0, step_len, step_w, h, "0.55 0.55 0.55 1", yaw=yaw)


def add_center_ridge_ramp(name, c, r, orient="h"):
    """
    添加图中“——”或“|”对应的双斜坡。
    它由两个半斜坡拼成，开口朝向地面，最高线在中间：
    - CENTER_RIDGE_H = 中间脊线高度
    - orient="h" 表示中线为横向（——）
    - orient="v" 表示中线为纵向（|）
    - 结构尽量占满整个格子
    """
    x, y = cell_center(c, r)
    ridge_len = 0.96 * CELL       # 沿中线方向基本占满整个格子
    total_span = 0.96 * CELL      # 垂直于中线方向基本占满整个格子
    half_span = total_span / 2.0
    thick = 0.04
    angle = math.atan2(CENTER_RIDGE_H, half_span)
    z = (half_span * abs(math.sin(angle)) + thick * abs(math.cos(angle))) / 2.0 + 0.005
    offset = (half_span / 2.0) * math.cos(angle)

    if orient == "h":
        # 中线沿 x 方向，两个半斜坡向格子上下两侧落地；中间脊线最高
        add_box(f"{name}_a", x, y + offset, z, ridge_len, half_span, thick, "0.38 0.38 0.38 1", roll=-angle)
        add_box(f"{name}_b", x, y - offset, z, ridge_len, half_span, thick, "0.38 0.38 0.38 1", roll= angle)
    else:
        # 中线沿 y 方向，两个半斜坡向格子左右两侧落地；中间脊线最高
        add_box(f"{name}_a", x + offset, y, z, half_span, ridge_len, thick, "0.38 0.38 0.38 1", pitch= angle)
        add_box(f"{name}_b", x - offset, y, z, half_span, ridge_len, thick, "0.38 0.38 0.38 1", pitch=-angle)


def build_world():
    # 地面
    add_box("floor", 0, 0, -0.025, 8.4 * CELL, 8.4 * CELL, 0.05, "0.95 0.95 0.95 1", collision=True)
    add_grid_visual()

    # 按最新标注图重建的墙体；其他物体不变
    # 水平墙段格式：(起始竖线c1, 结束竖线c2, 所在横线r)
    h_walls = [
        # 顶部区域
        (4, 6, 0),
        (3, 4, 1), (6, 7, 1),
        (2, 3, 2), (5, 6, 2), (7, 8, 2),

        # 中部区域
        (1, 3, 3), (5, 6, 3), (7, 8, 3),
        (0, 1, 4), (6, 7, 4),

        # 底部区域
        (5, 6, 5),
        (0, 1, 6), (4, 5, 6),
        (1, 3, 7),
    ]

    # 竖直墙段格式：(所在竖线c, 起始横线r1, 结束横线r2)
    v_walls = [
        # 顶部区域
        (4, 0, 1), (5, 0, 2), (6, 0, 1),
        (3, 1, 2),
        (7, 1, 4), (8, 2, 3),

        # 中部区域
        (2, 2, 3),
        (0, 4, 6), (1, 3, 4),
        (6, 2, 3), (6, 4, 5),

        # 底部区域
        (1, 6, 7),
        (4, 6, 7),
        (5, 5, 6),
    ]

    for i, (c1, c2, r) in enumerate(h_walls, 1):
        add_wall_h(f"wall_h_{i:02d}", c1, c2, r)
    for i, (c, r1, r2) in enumerate(v_walls, 1):
        add_wall_v(f"wall_v_{i:02d}", c, r1, r2)

    # “箱子”位置，按图中标注放置
    box_cells = [
        (4, 0),  # 上方箱子
        (3, 1),  # 左上箱子
        (7, 2),  # 右侧箱子
        (0, 4),  # 左侧箱子
        (4, 5),  # 中下箱子
        (1, 6),  # 左下箱子
    ]
    for i, (c, r) in enumerate(box_cells, 1):
        x, y = cell_center(c, r)
        add_box(f"box_{i}", x, y, 0.45, 0.58, 0.58, 0.90, "0.55 0.32 0.12 1")

    # 操作台
    x, y = cell_center(5, 5)
    add_box("operation_table", x, y, 0.35, 0.78, 0.65, 0.70, "0.25 0.35 0.55 1")

    # 斜坡与楼梯
    # 左上角两个斜坡是小斜坡，高度为 0.10 m
    add_small_ramp("ramp_up_left", 0, 5, "up")
    add_small_ramp("ramp_down_left", 1, 5, "down")
    # 中间这个高斜坡连接到 1.20 m 高的平台
    add_ramp("ramp_left_mid", 3, 3, "left")
    add_stairs("stairs_up", 1, 4, "up")

    # 图中数字“2”表示二层区域，不再用圆柱标记，而是生成高度为 SECOND_FLOOR_H 的二层平台
    second_floor_cells = [(1, 3), (2, 3), (2, 4)]
    for i, (c, r) in enumerate(second_floor_cells, 1):
        add_second_floor(f"second_floor_{i}", c, r)

    # 图中“——”与“|”不是低矮路障，而是两个 0.15 m 斜坡拼成的中脊斜坡
    h_barriers = [
        (5, 0), (5, 1), (6, 2),
        (5, 3), (3, 4), (4, 4), (5, 4),
        (2, 5), (3, 5), (2, 6), (3, 6),
    ]
    v_barriers = [(6, 1), (4, 3), (6, 3)]
    for i, (c, r) in enumerate(h_barriers, 1):
        add_center_ridge_ramp(f"ridge_h_{i:02d}", c, r, "h")
    for i, (c, r) in enumerate(v_barriers, 1):
        add_center_ridge_ramp(f"ridge_v_{i:02d}", c, r, "v")

    # 门口：绿色入口标记，不设置碰撞，方便机器人进入
    x, y = cell_center(3, 7)
    add_box("door_marker", x, y - 0.45 * CELL, 0.011, 0.75, 0.10, 0.02, "0.0 0.8 0.2 1", collision=False)


def write_world(out_path: str):
    build_world()
    sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='{WORLD_NAME}'>
    <gravity>0 0 -9.8</gravity>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <include>
      <uri>model://sun</uri>
    </include>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 -8 8 0 0.85 1.5708</pose>
      </camera>
    </gui>

{''.join(items)}

  </world>
</sdf>
"""
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(sdf)
    print(f"Generated: {out_path}")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    pkg = here.parent
    out = pkg / "worlds" / "image_map.world"
    write_world(str(out))
