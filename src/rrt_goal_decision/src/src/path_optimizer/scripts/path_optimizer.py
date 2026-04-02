#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf.transformations

class PathOptimizer:
    def __init__(self):
        rospy.init_node('path_optimizer', anonymous=False)
        
        # 优化参数
        self.cost_threshold = rospy.get_param('~cost_threshold', 50.0)
        self.dilate_iterations = rospy.get_param('~dilate_iterations', 1)  # 减少膨胀次数
        self.slice_interval = rospy.get_param('~slice_interval', 0.5)  # 增大切片间隔
        self.slice_width = rospy.get_param('~slice_width', 0.3)  # 减小搜索宽度
        self.noise_threshold = rospy.get_param('~noise_threshold', 0.2)
        self.smooth_window = rospy.get_param('~smooth_window', 5)  # 增大平滑窗口
        
        self.path_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback, queue_size=1)
        self.optimized_path_pub = rospy.Publisher('/move_base/TrajectoryPlannerROS/global_plan', Path, queue_size=1)
        self.debug_pub = rospy.Publisher('/path_optimizer/debug_mask', OccupancyGrid, queue_size=1)
        
        self.costmap = None
        self.costmap_resolution = 0.05
        self.costmap_origin = None
        
        rospy.loginfo("[PathOptimizer] Waiting for costmap...")
        self.costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback, queue_size=1)
        
        rospy.loginfo("[PathOptimizer] Path optimizer ready!")
    
    def costmap_callback(self, msg):
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = msg.info.origin.position
    
    def path_callback(self, msg):
        if len(msg.poses) < 3 or self.costmap is None:
            return
        
        start_time = rospy.Time.now()
        rospy.loginfo("[PathOptimizer] Received A* path: %d points", len(msg.poses))
        
        # 优化方案：简化版路径优化
        optimized_path = self.simple_optimize(msg.poses)
        
        output_msg = Path()
        output_msg.header = msg.header
        output_msg.poses = optimized_path
        self.optimized_path_pub.publish(output_msg)
        
        elapsed = (rospy.Time.now() - start_time).to_sec()
        rospy.loginfo("[PathOptimizer] Optimized: %d → %d points (%.2f ms)", 
                     len(msg.poses), len(optimized_path), elapsed * 1000)
    
    def simple_optimize(self, path):
        """简化版路径优化 - 基于代价梯度的横向偏移"""
        if len(path) < 3:
            return path
        
        optimized = []
        
        for i in range(0, len(path), 2):  # 降采样
            pose = path[i]
            
            # 计算当前点的代价梯度方向
            cx = int((pose.pose.position.x - self.costmap_origin.x) / self.costmap_resolution)
            cy = int((pose.pose.position.y - self.costmap_origin.y) / self.costmap_resolution)
            
            if not (0 <= cy < self.costmap.shape[0] and 0 <= cx < self.costmap.shape[1]):
                optimized.append(pose)
                continue
            
            # 计算 3x3 邻域的代价梯度
            gradient_x = 0
            gradient_y = 0
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= ny < self.costmap.shape[0] and 0 <= nx < self.costmap.shape[1]:
                        cost = self.costmap[ny, nx]
                        gradient_x += dx * cost
                        gradient_y += dy * cost
            
            # 如果当前点在低代价区域，不偏移
            if self.costmap[cy, cx] < self.cost_threshold:
                optimized.append(pose)
                continue
            
            # 沿梯度反方向偏移（远离高代价）
            norm = np.hypot(gradient_x, gradient_y)
            if norm > 0.01:
                offset_x = -gradient_x / norm * 0.2  # 偏移 20cm
                offset_y = -gradient_y / norm * 0.2
                
                new_pose = PoseStamped()
                new_pose.header = pose.header
                new_pose.pose.position.x = pose.pose.position.x + offset_x
                new_pose.pose.position.y = pose.pose.position.y + offset_y
                new_pose.pose.position.z = pose.pose.position.z
                new_pose.pose.orientation = pose.pose.orientation
                optimized.append(new_pose)
            else:
                optimized.append(pose)
        
        # 平滑处理
        return self.smooth_path(optimized)
    
    def smooth_path(self, path):
        """移动平均平滑"""
        if len(path) < 3:
            return path
        
        smoothed = []
        for i in range(len(path)):
            if i == 0 or i == len(path) - 1:
                smoothed.append(path[i])
                continue
            
            # 计算局部平均值
            window = min(self.smooth_window, i, len(path) - 1 - i)
            avg_x = sum(path[i+j].pose.position.x for j in range(-window, window+1)) / (2*window + 1)
            avg_y = sum(path[i+j].pose.position.y for j in range(-window, window+1)) / (2*window + 1)
            
            avg_pose = PoseStamped()
            avg_pose.header = path[i].header
            avg_pose.pose.position.x = avg_x
            avg_pose.pose.position.y = avg_y
            avg_pose.pose.position.z = path[i].pose.position.z
            
            # 计算朝向
            if i < len(path) - 1:
                dx = path[i+1].pose.position.x - path[i].pose.position.x
                dy = path[i+1].pose.position.y - path[i].pose.position.y
                yaw = np.arctan2(dy, dx)
                q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                avg_pose.pose.orientation.x = q[0]
                avg_pose.pose.orientation.y = q[1]
                avg_pose.pose.orientation.z = q[2]
                avg_pose.pose.orientation.w = q[3]
            else:
                avg_pose.pose.orientation = path[-1].pose.orientation
            
            smoothed.append(avg_pose)
        
        return smoothed
    
    def publish_debug_mask(self, mask, header):
        msg = OccupancyGrid()
        msg.header = header
        msg.info.width = mask.shape[1]
        msg.info.height = mask.shape[0]
        msg.info.resolution = self.costmap_resolution
        msg.info.origin.position.x = self.costmap_origin.x
        msg.info.origin.position.y = self.costmap_origin.y
        msg.data = (255 - mask).flatten().tolist()
        self.debug_pub.publish(msg)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        optimizer = PathOptimizer()
        optimizer.run()
    except rospy.ROSInterruptException:
        pass
