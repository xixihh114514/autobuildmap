#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ['CUDA_VISIBLE_DEVICES'] = ''
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['MKL_NUM_THREADS'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from ultralytics import YOLO
import message_filters


class VisualObstacleDetector:
    def __init__(self):
        rospy.init_node('visual_obstacle_detector', anonymous=True)

        rospy.loginfo("等待 /clock...")
        rospy.wait_for_message('/clock', rospy.msg.AnyMsg, timeout=30.0)

        # 扩展类：AGV/无人车常见障碍物 (COCO ID)
        self.classes = [0, 1, 2, 3, 4, 5, 6, 7, 39, 41, 46, 49, 56]  # person, bicycle, car, motorcycle, airplane, bus, train, truck, bottle, cup, bowl, suitcase, chair
        self.class_names = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 39: 'bottle', 41: 'cup', 46: 'bowl', 49: 'suitcase', 56: 'chair'}
        self.min_depth = 0.30
        self.max_depth = 8.0
        self.conf_thr = 0.60

        # 加载 YOLO
        model_path = "/home/rera/robotcup2026/src/visual_obstacle_detector/models/yolov8n.pt"
        rospy.loginfo(f"加载 YOLO 模型: {model_path}")
        self.model = YOLO(model_path)
        self.model.fuse()
        self.model.to('cpu')
        rospy.loginfo("YOLO 模型加载完成")

        # 等待原始内参
        rospy.loginfo("等待相机内参...")
        self.info = None
        while not rospy.is_shutdown() and self.info is None:
            for t in ['/camera/color/camera_info', '/camera/depth/camera_info']:
                try:
                    self.info = rospy.wait_for_message(t, CameraInfo, timeout=5.0)
                    rospy.loginfo(f"内参就绪 ← {t}  fx={self.info.K[0]:.1f}")
                    break
                except: pass
            rospy.sleep(0.5)

        self.bridge = CvBridge()
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, dep_sub], 10, 0.2)
        ts.registerCallback(self.callback)

        self.pub = rospy.Publisher('/visual_obstacle_cloud', PointCloud2, queue_size=2)
        rospy.loginfo("视觉障碍物检测节点启动完成 → AGV障碍物轮廓点云 + Z轴向前！")

    def callback(self, rgb_msg, dep_msg):
        try:
            # 原图 + 深度图
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(dep_msg, "passthrough")
            if raw_depth.ndim == 3:
                depth = raw_depth[:, :, 0].astype(np.float32) / 1000.0
            elif raw_depth.dtype == np.uint16:
                depth = raw_depth.astype(np.float32) / 1000.0
            else:
                depth = raw_depth.astype(np.float32)

            small_rgb = cv2.resize(rgb, (416, 416))
            results = self.model(small_rgb, imgsz=416, conf=self.conf_thr,
                                classes=self.classes, verbose=False)[0]

            rospy.loginfo(f"\n{'='*70}")
            rospy.loginfo(f"时间: {rospy.get_time():.1f}s  |  检测到 {len(results.boxes) if results.boxes else 0} 个障碍物")

            points = []

            if results.boxes is not None:
                for box in results.boxes:
                    cls_id = int(box.cls[0].item())
                    conf   = box.conf[0].item()
                    name   = self.class_names.get(cls_id, 'unknown')

                    # 映射框到原图 640x480
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    x1 = int(x1 * 640 / 416)
                    y1 = int(y1 * 480 / 416)
                    x2 = int(x2 * 640 / 416)
                    y2 = int(y2 * 480 / 416)

                    # 生成轮廓点云（均匀采样 100 个点）
                    num_points = 100
                    xs = np.linspace(x1 + 5, x2 - 5, 10)
                    ys = np.linspace(y1 + 5, y2 - 5, 12)
                    xv, yv = np.meshgrid(xs, ys)
                    pixels = np.column_stack((xv.ravel(), yv.ravel()))
                    np.random.shuffle(pixels)

                    valid = 0
                    for px, py in pixels[:num_points]:
                        px, py = int(px), int(py)
                        if py >= 480 or px >= 640: continue
                        d = depth[py, px]
                        if np.isnan(d) or not (self.min_depth < d < self.max_depth):
                            continue
                        X = (px - self.info.K[2]) * d / self.info.K[0]
                        Y = (py - self.info.K[5]) * d / self.info.K[4]
                        Z = d
                        points.append([X, Y, Z])
                        valid += 1

                    rospy.loginfo(f"  {name} | 置信度 {conf:.3f} | 生成 {valid} 个轮廓点")

            # 发布绿色点云
            if points:
                header = rgb_msg.header
                header.frame_id = "camera_depth_optical_frame"   # Z轴向前

                fields = [
                    PointField('x', 0,  PointField.FLOAT32, 1),
                    PointField('y', 4,  PointField.FLOAT32, 1),
                    PointField('z', 8,  PointField.FLOAT32, 1),
                    PointField('rgb',12, PointField.UINT32, 1),
                ]
                colored = [[p[0], p[1], p[2], (0 << 16) | (180 << 8) | 0] for p in points]
                cloud = pc2.create_cloud(header, fields, colored)
                self.pub.publish(cloud)
                rospy.loginfo(f"成功发布 {len(points)} 个红色轮廓点云！")
            else:
                rospy.logwarn("本帧无有效点云")

        except Exception as e:
            rospy.logerr_throttle(5, f"回调异常: {e}")
            import traceback; traceback.print_exc()


if __name__ == '__main__':
    try:
        VisualObstacleDetector()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("视觉检测节点关闭")