#!/usr/bin/env python3
"""Unpause Gazebo after spawn_model has applied its delayed -J positions."""

import time

import rospy
from std_srvs.srv import Empty


if __name__ == "__main__":
    rospy.init_node("unpause_after_spawn")
    rospy.wait_for_service("/gazebo/unpause_physics", timeout=20.0)
    # Wall time is intentional because /clock does not advance while paused.
    time.sleep(rospy.get_param("~delay", 2.5))
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
    rospy.loginfo("Gazebo physics unpaused after Go2-W joint initialization")
