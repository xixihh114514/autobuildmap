#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import cv2
import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from ultralytics import YOLO


class PersonGlobalLocalizer(object):
    def __init__(self):
        self.bridge = CvBridge()

        self.model_path = rospy.get_param(
            "~model_path",
            "/home/lzk/robotcup2026/src/visual_obstacle_detector/models/yolo.pt",
        )
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.conf_threshold = rospy.get_param("~conf_threshold", 0.4)
        self.iou_threshold = rospy.get_param("~iou_threshold", 0.45)
        self.sync_slop = rospy.get_param("~sync_slop", 0.1)
        self.depth_unit_scale = rospy.get_param("~depth_unit_scale", 0.001)
        self.depth_min = rospy.get_param("~depth_min", 0.2)
        self.depth_max = rospy.get_param("~depth_max", 8.0)
        self.depth_roi_half = rospy.get_param("~depth_roi_half", 4)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.3)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.transform_timeout = rospy.get_param("~transform_timeout", 0.05)
        self.min_confirm_frames = max(1, int(rospy.get_param("~min_confirm_frames", 3)))
        self.confirm_pixel_tolerance = rospy.get_param(
            "~confirm_pixel_tolerance", 60.0
        )
        self.confirm_timeout = rospy.get_param("~confirm_timeout", 0.5)
        self.person_classes = {
            str(name).strip().lower()
            for name in rospy.get_param("~person_classes", ["victim"])
        }
        self.pending_confirmations = []

        rospy.loginfo("Loading YOLO model from %s", self.model_path)
        self.model = YOLO(self.model_path)

        self.person_cloud_pub = rospy.Publisher(
            "~person_camera_cloud", PointCloud2, queue_size=1
        )
        self.person_map_cloud_pub = rospy.Publisher(
            "~person_map_cloud", PointCloud2, queue_size=1
        )
        self.marker_pub = rospy.Publisher("~person_markers", MarkerArray, queue_size=1)
        self.debug_image_pub = rospy.Publisher(
            "~debug_image", Image, queue_size=1
        ) if self.publish_debug_image else None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.color_sub = message_filters.Subscriber(self.color_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.synced_callback)

        rospy.loginfo(
            "person_global_localizer is ready, publishing person centers in %s after %d consecutive frames",
            self.global_frame,
            self.min_confirm_frames,
        )

    def synced_callback(self, color_msg, depth_msg, camera_info_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        depth_m = self.depth_to_meters(depth_image)
        if depth_m is None:
            rospy.logwarn_throttle(2.0, "Unsupported depth image dtype: %s", depth_image.dtype)
            return

        detections = self.run_yolo(color_image)
        detections = self.update_confirmations(detections, color_msg.header.stamp.to_sec())
        current_camera_points = []
        current_map_points = []
        marker_array = MarkerArray()
        debug_image = color_image.copy()
        camera_frame = depth_msg.header.frame_id
        confirmed_marker_id = 0

        for det in detections:
            bbox = det["bbox"]
            confidence = det["confidence"]
            confirm_count = det["confirm_count"]
            is_confirmed = det["is_confirmed"]
            newly_confirmed = det["newly_confirmed"]
            pixel_xy = self.bbox_center_pixel(bbox, depth_m.shape)
            depth_value = None
            camera_point = None

            if newly_confirmed:
                rospy.loginfo(
                    "Person detection confirmed after %d consecutive frames near pixel (%d, %d)",
                    confirm_count,
                    pixel_xy[0],
                    pixel_xy[1],
                )

            if is_confirmed:
                sample = self.sample_depth_and_point(depth_m, bbox, camera_info_msg)
                if sample is not None:
                    point_xyz, pixel_xy, depth_value = sample
                    camera_point = PointStamped()
                    camera_point.header = Header(
                        stamp=depth_msg.header.stamp,
                        frame_id=camera_frame,
                    )
                    camera_point.point.x = float(point_xyz[0])
                    camera_point.point.y = float(point_xyz[1])
                    camera_point.point.z = float(point_xyz[2])

                    current_camera_points.append(
                        (
                            camera_point.point.x,
                            camera_point.point.y,
                            camera_point.point.z,
                            confidence,
                        )
                    )
                    marker_array.markers.extend(
                        self.make_markers(
                            confirmed_marker_id,
                            camera_point,
                            confidence,
                            camera_frame,
                            confirm_count,
                        )
                    )
                    map_point = self.transform_point(camera_point)
                    if map_point is not None:
                        current_map_points.append(
                            (
                                map_point.point.x,
                                map_point.point.y,
                                map_point.point.z,
                                confidence,
                            )
                        )
                    confirmed_marker_id += 1

            self.draw_detection(
                debug_image,
                bbox,
                pixel_xy,
                confidence,
                depth_value,
                camera_point,
                confirmed_marker_id - 1 if camera_point is not None else -1,
                confirm_count,
                is_confirmed,
            )

        self.publish_cloud(
            self.person_cloud_pub,
            color_msg.header.stamp,
            camera_frame,
            current_camera_points,
        )
        self.publish_cloud(
            self.person_map_cloud_pub,
            color_msg.header.stamp,
            self.global_frame,
            current_map_points,
        )
        self.publish_markers(marker_array, color_msg.header.stamp, camera_frame)

        if self.debug_image_pub is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header = color_msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as exc:
                rospy.logwarn_throttle(2.0, "debug image publish failed: %s", exc)

    def run_yolo(self, color_image):
        results = self.model.predict(
            source=color_image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            verbose=False,
        )

        if not results:
            return []

        detections = []
        result = results[0]
        names = result.names if hasattr(result, "names") else {}
        boxes = result.boxes
        if boxes is None:
            return detections

        for box in boxes:
            cls_id = int(box.cls[0].item()) if box.cls is not None else -1
            raw_name = names.get(cls_id, "")
            cls_name = str(raw_name).strip().lower()
            if cls_name:
                is_person = cls_name in self.person_classes
            else:
                is_person = cls_id == 0

            if not is_person:
                continue

            conf = float(box.conf[0].item()) if box.conf is not None else 0.0
            xyxy = box.xyxy[0].cpu().numpy().astype(np.int32).tolist()
            detections.append(
                {
                    "bbox": xyxy,
                    "confidence": conf,
                }
            )

        return detections

    def update_confirmations(self, detections, stamp_sec):
        active_tracks = []
        used_previous_tracks = set()
        confirmed_detections = []

        for det in detections:
            center_xy = np.asarray(
                [
                    (det["bbox"][0] + det["bbox"][2]) * 0.5,
                    (det["bbox"][1] + det["bbox"][3]) * 0.5,
                ],
                dtype=np.float32,
            )

            matched_index = None
            matched_distance = None
            for index, track in enumerate(self.pending_confirmations):
                if index in used_previous_tracks:
                    continue

                time_gap = stamp_sec - track["stamp"]
                if time_gap > self.confirm_timeout:
                    continue

                pixel_shift = np.linalg.norm(center_xy - track["center"])
                if pixel_shift > self.confirm_pixel_tolerance:
                    continue

                if matched_distance is None or pixel_shift < matched_distance:
                    matched_index = index
                    matched_distance = pixel_shift

            if matched_index is None:
                confirm_count = 1
            else:
                confirm_count = self.pending_confirmations[matched_index]["count"] + 1
                used_previous_tracks.add(matched_index)

            active_tracks.append(
                {
                    "center": center_xy,
                    "count": confirm_count,
                    "stamp": stamp_sec,
                }
            )

            det = dict(det)
            det["confirm_count"] = confirm_count
            det["is_confirmed"] = confirm_count >= self.min_confirm_frames
            det["newly_confirmed"] = confirm_count == self.min_confirm_frames
            confirmed_detections.append(det)

        self.pending_confirmations = active_tracks
        return confirmed_detections

    def depth_to_meters(self, depth_image):
        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * float(self.depth_unit_scale)
        if depth_image.dtype in (np.float32, np.float64):
            return depth_image.astype(np.float32)
        return None

    def transform_point(self, point_stamped):
        try:
            return self.tf_buffer.transform(
                point_stamped,
                self.global_frame,
                rospy.Duration(self.transform_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "failed to transform person center from %s to %s: %s",
                point_stamped.header.frame_id,
                self.global_frame,
                exc,
            )
            return None

    def sample_depth_and_point(self, depth_m, bbox, camera_info_msg):
        h, w = depth_m.shape[:2]
        u, v = self.bbox_center_pixel(bbox, depth_m.shape)

        r = int(max(1, self.depth_roi_half))
        u0 = max(0, u - r)
        u1 = min(w, u + r + 1)
        v0 = max(0, v - r)
        v1 = min(h, v + r + 1)

        roi = depth_m[v0:v1, u0:u1]
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > self.depth_min) & (valid < self.depth_max)]
        if valid.size == 0:
            return None

        z = float(np.median(valid))
        fx = camera_info_msg.K[0]
        fy = camera_info_msg.K[4]
        cx = camera_info_msg.K[2]
        cy = camera_info_msg.K[5]
        if fx == 0.0 or fy == 0.0:
            return None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z), (u, v), z

    def bbox_center_pixel(self, bbox, image_shape):
        x1, y1, x2, y2 = bbox
        h, w = image_shape[:2]
        u = int(np.clip((x1 + x2) * 0.5, 0, w - 1))
        v = int(np.clip((y1 + y2) * 0.5, 0, h - 1))
        return u, v

    def make_markers(self, marker_id, camera_point, confidence, camera_frame, confirm_count):
        markers = []

        sphere = Marker()
        sphere.header.frame_id = camera_frame
        sphere.header.stamp = camera_point.header.stamp
        sphere.ns = "person_points"
        sphere.id = marker_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.orientation.w = 1.0
        sphere.pose.position = camera_point.point
        sphere.scale.x = 0.25
        sphere.scale.y = 0.25
        sphere.scale.z = 0.25
        sphere.color = ColorRGBA(1.0, 0.1, 0.1, 0.9)
        sphere.lifetime = rospy.Duration(self.marker_lifetime)
        markers.append(sphere)

        text = Marker()
        text.header.frame_id = camera_frame
        text.header.stamp = camera_point.header.stamp
        text.ns = "person_labels"
        text.id = 10000 + marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.x = camera_point.point.x
        text.pose.position.y = camera_point.point.y
        text.pose.position.z = camera_point.point.z + 0.45
        text.scale.z = 0.22
        text.color = ColorRGBA(0.1, 1.0, 0.1, 0.95)
        range_dist = math.sqrt(
            camera_point.point.x ** 2
            + camera_point.point.y ** 2
            + camera_point.point.z ** 2
        )
        text.text = "person {} {:.2f}m {:.2f} {}/{}".format(
            marker_id,
            range_dist,
            confidence,
            min(confirm_count, self.min_confirm_frames),
            self.min_confirm_frames,
        )
        text.lifetime = rospy.Duration(self.marker_lifetime)
        markers.append(text)

        return markers

    def draw_detection(
        self,
        image,
        bbox,
        pixel_xy,
        confidence,
        depth_value,
        camera_point,
        marker_id,
        confirm_count,
        is_confirmed,
    ):
        x1, y1, x2, y2 = bbox
        u, v = pixel_xy
        cv2.rectangle(image, (x1, y1), (x2, y2), (40, 220, 40), 2)
        cv2.circle(image, (u, v), 4, (0, 0, 255), -1)

        confirm_text = "{}/{}".format(
            min(confirm_count, self.min_confirm_frames),
            self.min_confirm_frames,
        )
        status_text = "ok" if is_confirmed else "wait"

        if camera_point is not None:
            camera_text = "id:{} conf:{:.2f} {} {} cam[{:.2f},{:.2f},{:.2f}]m".format(
                marker_id,
                confidence,
                status_text,
                confirm_text,
                camera_point.point.x,
                camera_point.point.y,
                camera_point.point.z,
            )
        else:
            camera_text = "conf:{:.2f} {} {}".format(
                confidence,
                status_text,
                confirm_text,
            )

        depth_text = (
            "depth:{:.2f}m".format(depth_value)
            if depth_value is not None
            else "depth:pending"
        )

        y_text_1 = max(20, y1 - 10)
        y_text_2 = min(image.shape[0] - 10, y_text_1 + 20)
        cv2.putText(
            image,
            camera_text,
            (x1, y_text_1),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (20, 255, 20),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            depth_text,
            (x1, y_text_2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (20, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def publish_markers(self, marker_array, stamp, frame_id):
        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = stamp
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.insert(0, delete_marker)
        self.marker_pub.publish(marker_array)

    def publish_cloud(self, publisher, stamp, frame_id, points):
        header = Header(stamp=stamp, frame_id=frame_id)
        cloud_msg = self.make_xyzi_cloud(header, points)
        publisher.publish(cloud_msg)

    def make_xyzi_cloud(self, header, points):
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        if not points:
            return pc2.create_cloud(header, fields, [])
        rows = [(float(x), float(y), float(z), float(i)) for x, y, z, i in points]
        return pc2.create_cloud(header, fields, rows)


def main():
    rospy.init_node("person_global_localizer", anonymous=False)
    PersonGlobalLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
