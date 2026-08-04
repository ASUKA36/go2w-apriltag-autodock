#!/usr/bin/env python3
"""Visual servoing for one AprilTag using apriltag_ros pose estimates."""

import math
import threading

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class TagFollower:
    def __init__(self):
        self.target_id = rospy.get_param("~target_id", 0)
        self.target_distance = rospy.get_param("~target_distance", 0.30)
        self.kp_linear = rospy.get_param("~kp_linear", 0.8)
        self.kp_angular = rospy.get_param("~kp_angular", 1.8)
        self.max_linear = rospy.get_param("~max_linear", 0.25)
        self.max_angular = rospy.get_param("~max_angular", 0.8)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.02)
        self.angle_tolerance = math.radians(rospy.get_param("~angle_tolerance_deg", 3.0))
        self.lost_timeout = rospy.get_param("~lost_timeout", 0.35)
        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.allow_reverse = rospy.get_param("~allow_reverse", True)
        self.rotate_in_place_angle = math.radians(
            rospy.get_param("~rotate_in_place_angle_deg", 18.0)
        )

        if self.target_distance <= 0.0 or self.control_rate <= 0.0:
            raise ValueError("target_distance and control_rate must be positive")

        self._lock = threading.Lock()
        self._last_seen = None
        self._tag_x = 0.0
        self._tag_z = 0.0
        self._stopped = True

        detections_topic = rospy.get_param("~detections_topic", "/tag_detections")
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self._publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._subscriber = rospy.Subscriber(
            detections_topic, AprilTagDetectionArray, self._detection_callback, queue_size=1
        )
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control)
        rospy.on_shutdown(self.stop)
        rospy.loginfo(
            "tag_follower: target id=%s, distance=%.3f m, output=%s",
            self.target_id,
            self.target_distance,
            cmd_vel_topic,
        )

    @staticmethod
    def _position(detection):
        # apriltag_ros uses PoseWithCovarianceStamped inside AprilTagDetection.
        return detection.pose.pose.pose.position

    def _detection_callback(self, message):
        candidates = []
        for detection in message.detections:
            ids = list(detection.id)
            if self.target_id in ids:
                position = self._position(detection)
                if position.z > 0.0 and math.isfinite(position.x) and math.isfinite(position.z):
                    candidates.append(position)

        if not candidates:
            return

        # If duplicate detections exist, use the closest valid target.
        position = min(candidates, key=lambda p: p.z)
        with self._lock:
            self._tag_x = position.x
            self._tag_z = position.z
            self._last_seen = rospy.Time.now()

    def _control(self, _event):
        now = rospy.Time.now()
        with self._lock:
            last_seen = self._last_seen
            x = self._tag_x
            z = self._tag_z

        if last_seen is None or (now - last_seen).to_sec() > self.lost_timeout:
            self.stop()
            return

        # In an optical frame: +x is image-right and +z points forward.
        bearing = math.atan2(x, z)
        distance_error = z - self.target_distance

        angular = -self.kp_angular * bearing
        if abs(bearing) <= self.angle_tolerance:
            angular = 0.0
        angular = clamp(angular, -self.max_angular, self.max_angular)

        linear = self.kp_linear * distance_error
        if abs(distance_error) <= self.distance_tolerance:
            linear = 0.0
        if abs(bearing) >= self.rotate_in_place_angle:
            linear = 0.0
        if not self.allow_reverse:
            linear = max(0.0, linear)
        linear = clamp(linear, -self.max_linear, self.max_linear)

        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._publisher.publish(command)
        self._stopped = linear == 0.0 and angular == 0.0

    def stop(self):
        if not self._stopped:
            self._publisher.publish(Twist())
            self._stopped = True


if __name__ == "__main__":
    rospy.init_node("tag_follower")
    try:
        TagFollower()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logerr("tag_follower stopped: %s", error)
