#!/usr/bin/env python3
"""Publish an OccupancyGrid matching the lightweight Gazebo world."""

import math
import rospy
from nav_msgs.msg import OccupancyGrid


def main():
    width_m = float(rospy.get_param("~map_width", 12.0))
    height_m = float(rospy.get_param("~map_height", 10.0))
    resolution = float(rospy.get_param("~map_resolution", 0.05))
    origin_x = float(rospy.get_param("~map_origin_x", -6.0))
    origin_y = float(rospy.get_param("~map_origin_y", -5.0))
    wall = float(rospy.get_param("~wall_thickness", 0.12))
    obstacles = rospy.get_param("~obstacles", [])
    width = int(math.ceil(width_m / resolution))
    height = int(math.ceil(height_m / resolution))
    data = [0] * (width * height)
    for gy in range(height):
        for gx in range(width):
            x = origin_x + (gx + 0.5) * resolution
            y = origin_y + (gy + 0.5) * resolution
            occupied = (
                x <= origin_x + wall or x >= origin_x + width_m - wall
                or y <= origin_y + wall or y >= origin_y + height_m - wall
                or any(r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in obstacles)
            )
            if occupied:
                data[gy * width + gx] = 100
    publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.info.resolution = resolution
    message.info.width = width
    message.info.height = height
    message.info.origin.position.x = origin_x
    message.info.origin.position.y = origin_y
    message.info.origin.orientation.w = 1.0
    message.data = data
    publisher.publish(message)
    rospy.loginfo("Published Gazebo world map: %dx%d", width, height)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("world_map")
    main()
