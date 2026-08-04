#!/usr/bin/env python3
"""End-to-end interface and motion smoke test for go2w_sim."""

import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState, LaserScan


def wait_for(topic, message_type, timeout=8.0):
    try:
        return rospy.wait_for_message(topic, message_type, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for %s", topic)
        return None


def main():
    occupancy = wait_for("/map", OccupancyGrid)
    scan = wait_for("/scan", LaserScan)
    joints = wait_for("/joint_states", JointState)
    start = wait_for("/odom", Odometry)
    if None in (occupancy, scan, joints, start):
        return 1
    if len(scan.ranges) < 180 or len(joints.name) != 16:
        rospy.logerr("Invalid sensor/model output: beams=%d joints=%d", len(scan.ranges), len(joints.name))
        return 1

    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    deadline = rospy.Time.now() + rospy.Duration(1.5)
    command = Twist()
    command.linear.x = 0.45
    command.angular.z = 0.15
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        publisher.publish(command)
        rate.sleep()
    publisher.publish(Twist())
    end = wait_for("/odom", Odometry, 3.0)
    if end is None:
        return 1
    dx = end.pose.pose.position.x - start.pose.pose.position.x
    dy = end.pose.pose.position.y - start.pose.pose.position.y
    travelled = math.hypot(dx, dy)
    if travelled < 0.25:
        rospy.logerr("Robot moved only %.3f m", travelled)
        return 1
    rospy.loginfo(
        "PASS: map=%dx%d, scan=%d beams, joints=%d, travelled=%.3f m",
        occupancy.info.width, occupancy.info.height, len(scan.ranges), len(joints.name), travelled,
    )
    return 0


if __name__ == "__main__":
    rospy.init_node("go2w_sim_smoke_test")
    sys.exit(main())
