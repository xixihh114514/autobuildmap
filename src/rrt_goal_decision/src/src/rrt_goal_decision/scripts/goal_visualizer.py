#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from move_base_msgs.msg import MoveBaseActionGoal
from matplotlib.lines import Line2D
from matplotlib import colors as mcolors
import math

class GoalVisualizer:

    def __init__(self):
        rospy.init_node("goal_debug_viewer", anonymous=True)

        self.map_data = None
        self.map_info = None
        self.goals = []
        self.active_goal_pos = None
        self.rescue_info = None

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/goal_candidates", Float32MultiArray, self.goal_callback)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_topic_callback)
        rospy.Subscriber("/rescue_mode_info", Float32MultiArray, self.rescue_info_callback)

        plt.ion()
        self.fig = plt.figure(figsize=(15, 8))
        self.ax_map = self.fig.add_subplot(1, 2, 1)
        self.ax_text = self.fig.add_subplot(1, 2, 2)
        self.ax_text.axis('off')

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def goal_callback(self, msg):
        data = msg.data
        new_goals = []
        
        # 【修复点 1】V5.0 数据格式变更：去掉了 cost 字段，现在每组数据只有 7 个
        step = 7 
        
        if len(data) % step != 0:
            rospy.logwarn("Received goal data length is not a multiple of %d", step)
            return

        for i in range(0, len(data), step):
            try:
                new_goals.append({
                    'id': int(data[i]),
                    'x': data[i+1],
                    'y': data[i+2],
                    'num': data[i+3],
                    'dist': data[i+4],
                    # 'cost': data[i+5],  # V5.0 已移除，不再读取
                    'area': data[i+5],   # area 索引前移
                    'weight': data[i+6]  # weight 索引前移
                })
            except IndexError:
                break
        
        self.goals = new_goals

    def goal_topic_callback(self, msg):
        try:
            p = msg.goal.target_pose.pose.position
            self.active_goal_pos = (p.x, p.y)
        except: pass

    def rescue_info_callback(self, msg):
        data = msg.data
        if len(data) < 9:
            return

        self.rescue_info = {
            'active': data[0] > 0.5,
            'elapsed': data[1],
            'remaining': data[2],
            'goal_active': data[3] > 0.5,
            'goal_x': data[4],
            'goal_y': data[5],
            'center_x': data[6],
            'center_y': data[7],
            'failures': data[8],
        }

    def run(self):
        rate = rospy.Rate(5)
        VIEW_SIZE = 24.0
        INVALID_WEIGHT = -1e9

        # 定义颜色渐变：黄 (低权重) -> 红 (高权重)
        low_color = np.array([1.0, 1.0, 0.0]) 
        high_color = np.array([1.0, 0.0, 0.0])

        while not rospy.is_shutdown():
            if self.map_data is None:
                rate.sleep()
                continue

            self.ax_map.clear()
            self.ax_text.clear()
            self.ax_text.axis('off')

            # --- 1. 绘制地图 ---
            info = self.map_info
            res = info.resolution
            ox, oy = info.origin.position.x, info.origin.position.y
            w, h = info.width, info.height
            
            cx, cy = ox + w*res/2, oy + h*res/2
            half = VIEW_SIZE / 2
            xmin, xmax = cx - half, cx + half
            ymin, ymax = cy - half, cy + half
            
            x0, y0 = max(0, int((xmin-ox)/res)), max(0, int((ymin-oy)/res))
            x1, y1 = min(w, int((xmax-ox)/res)), min(h, int((ymax-oy)/res))
            
            real_xmin, real_xmax, real_ymin, real_ymax = 0, 1, 0, 1

            if x0 < x1 and y0 < y1:
                real_xmin = ox + x0*res
                real_xmax = ox + x1*res
                real_ymin = oy + y0*res
                real_ymax = oy + y1*res
                
                self.ax_map.imshow(
                    self.map_data[y0:y1, x0:x1], cmap='gray', origin='lower',
                    extent=[real_xmin, real_xmax, real_ymin, real_ymax], aspect='equal'
                )
                self.ax_map.set_xlim(real_xmin, real_xmax)
                self.ax_map.set_ylim(real_ymin, real_ymax)

            # --- 2. 预处理：找出有效权重的最大最小值 ---
            valid_weights = [g['weight'] for g in self.goals if abs(g['weight'] - INVALID_WEIGHT) > 1e-5]
            
            min_w = min(valid_weights) if valid_weights else 0
            max_w = max(valid_weights) if valid_weights else 1
            
            # 防止除零错误
            weight_range = (max_w - min_w) if (max_w - min_w) > 1e-6 else 1.0

            table_rows = []
            
            for g in self.goals:
                x, y, w_val = g['x'], g['y'], g['weight']
                pid = g['id']
                
                # 视野裁剪
                if x < real_xmin or x > real_xmax or y < real_ymin or y > real_ymax:
                    continue

                # 状态判断
                is_active = False
                if self.active_goal_pos:
                    if math.hypot(x - self.active_goal_pos[0], y - self.active_goal_pos[1]) < 0.5:
                        is_active = True
                
                is_invalid = (abs(w_val - INVALID_WEIGHT) < 1e-5)

                # --- 计算颜色 ---
                point_color = 'k' # 默认黑
                
                if is_active:
                    point_color = 'green'
                    marker_edge = 'black'
                elif is_invalid:
                    point_color = 'lightgray'
                    marker_edge = 'red'
                else:
                    t = (w_val - min_w) / weight_range
                    t = max(0.0, min(1.0, t))
                    
                    r = low_color[0] + t * (high_color[0] - low_color[0])
                    g_comp = low_color[1] + t * (high_color[1] - low_color[1])
                    b = low_color[2] + t * (high_color[2] - low_color[2])
                    
                    point_color = (r, g_comp, b)
                    marker_edge = 'black'

                # 绘图
                if is_invalid:
                    self.ax_map.plot(x, y, 'o', markersize=6, color=point_color, markeredgecolor=marker_edge, markeredgewidth=1)
                    self.ax_map.text(x, y+0.2, str(pid), color='red', ha='center', fontsize=9, weight='bold')
                elif is_active:
                    self.ax_map.plot(x, y, 'o', markersize=10, color=point_color, markeredgecolor=marker_edge, markeredgewidth=2)
                    self.ax_map.text(x, y+0.3, str(pid), color='darkgreen', ha='center', fontsize=12, weight='bold')
                else:
                    self.ax_map.plot(x, y, 'o', markersize=6, color=point_color, markeredgecolor=marker_edge, markeredgewidth=1)
                    self.ax_map.text(x, y+0.2, str(pid), color='blue', ha='center', fontsize=9, weight='bold')

                status = "GOAL" if is_active else ("INVALID" if is_invalid else "OK")
                # 【修复点 2】移除了表格中的 Cost 数据列
                table_rows.append([
                    str(pid), status, 
                    f"{w_val:.2f}", 
                    f"{g['num']:.2f}", 
                    f"{g['dist']:.2f}", 
                    # f"{g['cost']:.2f}",  # 这一行已删除
                    f"{g['area']:.2f}"
                ])

            # 按总权重降序排序
            if table_rows:
                table_rows.sort(key=lambda r: float(r[2]), reverse=True)

            # --- 3. 绘制右侧表格 ---
            if table_rows:
                # 【修复点 3】移除了表头中的 'Cost' 列
                cols = ['ID', 'Status', 'Total', 'Num', 'Dist', 'Area']
                table = self.ax_text.table(
                    cellText=table_rows,
                    colLabels=cols,
                    loc='center',
                    cellLoc='center'
                )
                table.auto_set_font_size(False)
                table.set_fontsize(10)
                table.scale(1.2, 1.5)
                
                for (r, c), cell in table.get_celld().items():
                    if r == 0:
                        cell.set_facecolor('#dddddd')
                        cell.set_text_props(weight='bold')
                    else:
                        status = table_rows[r-1][1]
                        if status == "GOAL":
                            cell.set_facecolor('#d4edda')
                        elif status == "INVALID":
                            cell.set_facecolor('#f8d7da')
                        else:
                            cell.set_facecolor('#ffffff')
            
            rescue_status = "Rescue: OFF"
            if self.rescue_info and self.rescue_info['active']:
                rescue_status = (
                    "Rescue: ON | "
                    f"elapsed {self.rescue_info['elapsed']:.1f}s | "
                    f"remaining {self.rescue_info['remaining']:.1f}s | "
                    f"goal ({self.rescue_info['goal_x']:.2f}, {self.rescue_info['goal_y']:.2f})"
                )

            self.ax_text.set_title(
                f"Real-time Weight Details (Sorted by Total)\n{rescue_status}",
                fontsize=12,
                pad=10,
            )
            
            # 更新标题说明
            self.ax_map.set_title(f"Map View (Yellow=Low Weight, Red=High Weight, Green=Active)")

            plt.pause(0.01)
            rate.sleep()

if __name__ == "__main__":
    try:
        gv = GoalVisualizer()
        gv.run()
    except rospy.ROSInterruptException:
        pass
