#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import message_filters
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


class QRGlobalLocalizer(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        if hasattr(self.qr_detector, "setUseAlignmentMarkers"):
            self.qr_detector.setUseAlignmentMarkers(True)

        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.sync_slop = rospy.get_param("~sync_slop", 0.1)
        self.depth_unit_scale = rospy.get_param("~depth_unit_scale", 0.001)
        self.depth_min = rospy.get_param("~depth_min", 0.2)
        self.depth_max = rospy.get_param("~depth_max", 8.0)
        self.depth_roi_half = rospy.get_param("~depth_roi_half", 4)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.3)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.transform_timeout = rospy.get_param("~transform_timeout", 0.05)

        self.map_marker_pub = rospy.Publisher(
            "~qr_map_markers", MarkerArray, queue_size=1
        )
        self.debug_image_pub = (
            rospy.Publisher("~debug_image", Image, queue_size=1)
            if self.publish_debug_image
            else None
        )

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
            "qr_global_localizer is ready, publishing QR map markers in %s",
            self.global_frame,
        )

    def synced_callback(self, color_msg, depth_msg, camera_info_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        depth_m = self.depth_to_meters(depth_image)
        if depth_m is None:
            rospy.logwarn_throttle(
                2.0, "Unsupported depth image dtype: %s", depth_image.dtype
            )
            return

        detections = self.detect_qrs(color_image)
        marker_array = MarkerArray()
        debug_image = color_image.copy()
        camera_frame = depth_msg.header.frame_id

        for marker_id, det in enumerate(detections):
            qr_text = det["text"]
            corners = det["corners"]
            center = np.mean(corners, axis=0)

            sample = self.sample_depth_and_point(depth_m, center, camera_info_msg)
            if sample is None:
                self.draw_detection(debug_image, corners, qr_text, None, None)
                continue

            point_xyz, pixel_xy, depth_value = sample
            camera_point = PointStamped()
            camera_point.header = Header(
                stamp=depth_msg.header.stamp,
                frame_id=camera_frame,
            )
            camera_point.point.x = float(point_xyz[0])
            camera_point.point.y = float(point_xyz[1])
            camera_point.point.z = float(point_xyz[2])

            map_point = self.transform_point(camera_point)
            if map_point is not None:
                marker_array.markers.extend(
                    self.make_map_markers(
                        marker_id,
                        map_point,
                        self.make_qr_label(qr_text),
                    )
                )

            self.draw_detection(
                debug_image,
                corners,
                qr_text,
                pixel_xy,
                depth_value,
            )

        self.publish_markers(marker_array, color_msg.header.stamp, self.global_frame)

        if self.debug_image_pub is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header = color_msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as exc:
                rospy.logwarn_throttle(2.0, "debug image publish failed: %s", exc)

    def detect_qrs(self, color_image):
        detections = []

        if hasattr(self.qr_detector, "detectAndDecodeMulti"):
            try:
                retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(
                    color_image
                )
            except cv2.error as exc:
                rospy.logwarn_throttle(2.0, "detectAndDecodeMulti failed: %s", exc)
                retval, decoded_info, points = False, [], None

            if retval and points is not None:
                for text, corners in zip(decoded_info, points):
                    normalized_text = self.normalize_qr_text(text)
                    if not normalized_text:
                        continue
                    detections.append(
                        {
                            "text": normalized_text,
                            "corners": np.asarray(corners, dtype=np.float32).reshape(-1, 2),
                        }
                    )

        if detections:
            return detections

        try:
            text, points, _ = self.qr_detector.detectAndDecode(color_image)
        except cv2.error as exc:
            rospy.logwarn_throttle(2.0, "detectAndDecode failed: %s", exc)
            return detections

        normalized_text = self.normalize_qr_text(text)
        if normalized_text and points is not None:
            detections.append(
                {
                    "text": normalized_text,
                    "corners": np.asarray(points, dtype=np.float32).reshape(-1, 2),
                }
            )

        return detections

    def normalize_qr_text(self, text):
        if text is None:
            return ""
        raw_text = str(text)
        printable_text = "".join(ch for ch in raw_text if ch.isprintable())
        normalized_text = " ".join(printable_text.split())

        if normalized_text != raw_text:
            rospy.logwarn_throttle(
                2.0,
                "QR decoded text normalized from %s to %s",
                repr(raw_text),
                repr(normalized_text),
            )

        return normalized_text

    def make_qr_label(self, qr_text):
        return "cn{}".format(qr_text)

    def depth_to_meters(self, depth_image):
        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * float(self.depth_unit_scale)
        if depth_image.dtype in (np.float32, np.float64):
            return depth_image.astype(np.float32)
        return None

    def sample_depth_and_point(self, depth_m, center_xy, camera_info_msg):
        h, w = depth_m.shape[:2]
        u = int(np.clip(round(center_xy[0]), 0, w - 1))
        v = int(np.clip(round(center_xy[1]), 0, h - 1))

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
                "failed to transform QR center from %s to %s: %s",
                point_stamped.header.frame_id,
                self.global_frame,
                exc,
            )
            return None

    def make_map_markers(self, marker_id, map_point, label):
        markers = []

        sphere = Marker()
        sphere.header.frame_id = self.global_frame
        sphere.header.stamp = map_point.header.stamp
        sphere.ns = "qr_points_map"
        sphere.id = marker_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.orientation.w = 1.0
        sphere.pose.position = map_point.point
        sphere.scale.x = 0.22
        sphere.scale.y = 0.22
        sphere.scale.z = 0.22
        sphere.color = ColorRGBA(0.1, 0.6, 1.0, 0.9)
        sphere.lifetime = rospy.Duration(self.marker_lifetime)
        markers.append(sphere)

        text = Marker()
        text.header.frame_id = self.global_frame
        text.header.stamp = map_point.header.stamp
        text.ns = "qr_labels_map"
        text.id = 10000 + marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.x = map_point.point.x
        text.pose.position.y = map_point.point.y
        text.pose.position.z = map_point.point.z + 0.40
        text.scale.z = 0.22
        text.color = ColorRGBA(1.0, 1.0, 1.0, 0.95)
        text.text = label
        text.lifetime = rospy.Duration(self.marker_lifetime)
        markers.append(text)

        return markers

    def draw_detection(self, image, corners, qr_text, pixel_xy, depth_value):
        polygon = np.asarray(corners, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(image, [polygon], isClosed=True, color=(255, 180, 0), thickness=2)

        if pixel_xy is not None:
            cv2.circle(image, tuple(pixel_xy), 4, (0, 0, 255), -1)

        label_text = self.make_qr_label(qr_text)
        x_text = int(np.min(corners[:, 0]))
        y_text_1 = max(20, int(np.min(corners[:, 1])) - 10)
        y_text_2 = min(image.shape[0] - 10, y_text_1 + 20)
        cv2.putText(
            image,
            label_text,
            (x_text, y_text_1),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 220, 80),
            1,
            cv2.LINE_AA,
        )

        if depth_value is not None:
            depth_text = "depth:{:.2f}m".format(depth_value)
            cv2.putText(
                image,
                depth_text,
                (x_text, y_text_2),
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
        self.map_marker_pub.publish(marker_array)


def main():
    rospy.init_node("qr_global_localizer", anonymous=False)
    QRGlobalLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
