#!/usr/bin/env python3

import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class PointCloudInspector(object):
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/camera/depth/points")
        self.once = rospy.get_param("~once", True)
        self.max_points_to_check = int(rospy.get_param("~max_points_to_check", 0))
        self.message_count = 0
        self.sub = rospy.Subscriber(self.topic, PointCloud2, self.callback, queue_size=1)
        rospy.loginfo("Listening on %s", self.topic)

    def callback(self, msg):
        self.message_count += 1
        total_points = int(msg.width) * int(msg.height)
        organized = msg.height > 1

        valid_points = 0
        nan_points = 0
        finite_xyz_points = 0

        points_iter = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        for idx, (x, y, z) in enumerate(points_iter):
            if self.max_points_to_check > 0 and idx >= self.max_points_to_check:
                break
            has_nan = math.isnan(x) or math.isnan(y) or math.isnan(z)
            if has_nan:
                nan_points += 1
            else:
                finite_xyz_points += 1
                if not (x == 0.0 and y == 0.0 and z == 0.0):
                    valid_points += 1

        inspected_points = total_points if self.max_points_to_check <= 0 else min(total_points, self.max_points_to_check)

        rospy.loginfo("----- PointCloud2 message #%d -----", self.message_count)
        rospy.loginfo("topic=%s", self.topic)
        rospy.loginfo("frame_id=%s", msg.header.frame_id)
        rospy.loginfo("stamp=%.6f", msg.header.stamp.to_sec())
        rospy.loginfo("width=%d height=%d total_points=%d", msg.width, msg.height, total_points)
        rospy.loginfo("is_dense=%s point_step=%d row_step=%d", msg.is_dense, msg.point_step, msg.row_step)
        rospy.loginfo("organized=%s", organized)
        rospy.loginfo("inspected_points=%d finite_xyz=%d nan_xyz=%d nonzero_xyz=%d",
                      inspected_points, finite_xyz_points, nan_points, valid_points)

        if organized:
            rospy.loginfo("Interpretation: organized point cloud detected because height > 1.")
        else:
            rospy.loginfo("Interpretation: unordered point cloud detected because height == 1.")

        if self.once:
            rospy.signal_shutdown("Inspection finished")


def main():
    rospy.init_node("check_ordered_pc", anonymous=False)
    PointCloudInspector()
    rospy.spin()


if __name__ == "__main__":
    main()
