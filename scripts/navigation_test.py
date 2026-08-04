#!/usr/bin/env python3
"""Verify that move_base can plan and drive to a goal in nav_sim."""

import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def main():
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(15.0)):
        rospy.logerr("move_base action server did not start")
        return 1

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -1.8
    goal.target_pose.pose.position.y = -3.5
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(35.0)):
        client.cancel_goal()
        rospy.logerr("Navigation goal timed out")
        return 1
    if client.get_state() != GoalStatus.SUCCEEDED:
        rospy.logerr("Navigation failed with action state %d", client.get_state())
        return 1
    rospy.loginfo("PASS: move_base reached the test goal")
    return 0


if __name__ == "__main__":
    rospy.init_node("nav_sim_navigation_test")
    sys.exit(main())
