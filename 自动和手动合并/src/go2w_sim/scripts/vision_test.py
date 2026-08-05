#!/usr/bin/env python3
"""Check that the Gazebo RGB-D camera publishes valid images and intrinsics."""

import sys
import rospy
from sensor_msgs.msg import CameraInfo, Image


def receive(topic, message_type, timeout=15.0):
    try:
        return rospy.wait_for_message(topic, message_type, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for %s", topic)
        return None


def main():
    color = receive("/camera/color/image_raw", Image)
    color_info = receive("/camera/color/camera_info", CameraInfo)
    depth = receive("/camera/depth/image_raw", Image)
    depth_info = receive("/camera/depth/camera_info", CameraInfo)
    if any(message is None for message in (color, color_info, depth, depth_info)):
        return 1
    if color.width != 424 or color.height != 240 or not color.data:
        rospy.logerr("Invalid RGB image: %dx%d bytes=%d", color.width, color.height, len(color.data))
        return 1
    if depth.width != 424 or depth.height != 240 or not depth.data:
        rospy.logerr("Invalid depth image: %dx%d bytes=%d", depth.width, depth.height, len(depth.data))
        return 1
    if color_info.K[0] <= 0.0 or depth_info.K[0] <= 0.0:
        rospy.logerr("Camera intrinsics are invalid")
        return 1
    rospy.loginfo(
        "PASS: RGB=%dx%d %s, depth=%dx%d %s, fx=%.1f",
        color.width, color.height, color.encoding,
        depth.width, depth.height, depth.encoding, color_info.K[0],
    )
    return 0


if __name__ == "__main__":
    rospy.init_node("go2w_sim_vision_test")
    sys.exit(main())
