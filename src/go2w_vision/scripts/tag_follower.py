#!/usr/bin/env python3
"""Visual servoing for one AprilTag using apriltag_ros pose estimates."""

import math
import threading

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class TagFollower:
    def __init__(self):
        self.target_id = rospy.get_param("~target_id", 0)
        self.target_distance = rospy.get_param("~target_distance", 0.30)
        self.kp_linear = rospy.get_param("~kp_linear", 0.8)
        self.kp_angular = rospy.get_param("~kp_angular", 1.8)
        self.kp_facing = rospy.get_param("~kp_facing", 1.2)
        self.kp_lateral = rospy.get_param("~kp_lateral", 0.8)
        self.max_linear = rospy.get_param("~max_linear", 0.25)
        self.max_angular = rospy.get_param("~max_angular", 0.8)
        self.max_lateral = rospy.get_param("~max_lateral", 0.15)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.02)
        self.distance_hysteresis = rospy.get_param("~distance_hysteresis", 0.03)
        self.angle_tolerance = math.radians(rospy.get_param("~angle_tolerance_deg", 3.0))
        self.angle_hysteresis = math.radians(rospy.get_param("~angle_hysteresis_deg", 2.0))
        self.facing_tolerance = math.radians(
            rospy.get_param("~facing_tolerance_deg", 7.0)
        )
        self.facing_hysteresis = math.radians(
            rospy.get_param("~facing_hysteresis_deg", 3.0)
        )
        self.orientation_start_distance = rospy.get_param(
            "~orientation_start_distance", 0.70
        )
        self.use_orientation_alignment = rospy.get_param(
            "~use_orientation_alignment", True
        )
        self.use_lateral_alignment = rospy.get_param("~use_lateral_alignment", True)
        self.lost_timeout = rospy.get_param("~lost_timeout", 0.35)
        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.allow_reverse = rospy.get_param("~allow_reverse", True)
        self.rotate_in_place_angle = math.radians(
            rospy.get_param("~rotate_in_place_angle_deg", 18.0)
        )
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.35)
        self.arrival_hold_time = rospy.get_param("~arrival_hold_time", 1.0)
        self.arrival_dropout_timeout = rospy.get_param("~arrival_dropout_timeout", 1.5)
        self.auto_lie_down = rospy.get_param("~auto_lie_down", True)

        if self.target_distance <= 0.0 or self.control_rate <= 0.0:
            raise ValueError("target_distance and control_rate must be positive")

        self._lock = threading.Lock()
        self._last_seen = None
        self._tag_x = 0.0
        self._tag_z = 0.0
        self._facing_error = 0.0
        self._stopped = True
        self._arrival_started = None
        self._completed = False

        detections_topic = rospy.get_param("~detections_topic", "/tag_detections")
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self._publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        posture_topic = rospy.get_param("~posture_topic", "/go2w/posture_cmd")
        self._posture_publisher = rospy.Publisher(posture_topic, String, queue_size=1, latch=True)
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

    @staticmethod
    def _facing_error_from_orientation(detection):
        """Return horizontal angle between tag normal and camera forward.

        apriltag_ros expresses the tag frame in the camera optical frame. The
        tag's +Z normal usually points toward the camera, so front-on is
        (nx, nz) = (0, -1). Flip an ambiguous PnP normal when needed.
        """
        q = detection.pose.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-9:
            return None
        x, y, z, w = (value / norm for value in values)
        normal_x = 2.0 * (x * z + y * w)
        normal_z = 1.0 - 2.0 * (x * x + y * y)
        if normal_z > 0.0:
            normal_x = -normal_x
            normal_z = -normal_z
        return math.atan2(normal_x, -normal_z)

    def _detection_callback(self, message):
        candidates = []
        for detection in message.detections:
            ids = list(detection.id)
            if self.target_id in ids:
                position = self._position(detection)
                if position.z > 0.0 and math.isfinite(position.x) and math.isfinite(position.z):
                    facing_error = self._facing_error_from_orientation(detection)
                    if facing_error is not None:
                        candidates.append((position, facing_error))

        if not candidates:
            return

        # If duplicate detections exist, use the closest valid target.
        position, facing_error = min(candidates, key=lambda item: item[0].z)
        with self._lock:
            if self._last_seen is None:
                self._tag_x, self._tag_z = position.x, position.z
                self._facing_error = facing_error
            else:
                a = clamp(self.filter_alpha, 0.0, 1.0)
                self._tag_x = a * position.x + (1.0 - a) * self._tag_x
                self._tag_z = a * position.z + (1.0 - a) * self._tag_z
                delta = wrap_angle(facing_error - self._facing_error)
                self._facing_error = wrap_angle(self._facing_error + a * delta)
            self._last_seen = rospy.Time.now()

    def _control(self, _event):
        now = rospy.Time.now()
        with self._lock:
            last_seen = self._last_seen
            x = self._tag_x
            z = self._tag_z
            facing_error = self._facing_error
            completed = self._completed

        if completed:
            self.stop()
            return

        unseen_time = float("inf") if last_seen is None else (now - last_seen).to_sec()
        if last_seen is None or unseen_time > self.lost_timeout:
            self.stop()
            # Once the robot is confirmed inside the goal zone it remains
            # stopped. Permit a short close-range detection dropout while the
            # hold timer finishes, but never continue driving without vision.
            if self._arrival_started is not None and unseen_time <= self.arrival_dropout_timeout:
                if (now - self._arrival_started).to_sec() >= self.arrival_hold_time:
                    self._finish_approach()
            else:
                self._arrival_started = None
            return

        # In an optical frame: +x is image-right and +z points forward.
        bearing = math.atan2(x, z)
        distance_error = z - self.target_distance

        alignment_weight = 0.0
        if self.use_orientation_alignment and z < self.orientation_start_distance:
            span = max(0.10, self.orientation_start_distance - self.target_distance)
            alignment_weight = clamp(
                (self.orientation_start_distance - z) / span, 0.0, 1.0
            )

        # Far away, steer primarily by image bearing. Near the goal, also make
        # the camera optical axis perpendicular to the tag plane.
        angular = (
            -self.kp_angular * bearing
            + alignment_weight * self.kp_facing * facing_error
        )
        if abs(bearing) <= self.angle_tolerance:
            angular = alignment_weight * self.kp_facing * facing_error
        if (
            abs(bearing) <= self.angle_tolerance
            and (not self.use_orientation_alignment or abs(facing_error) <= self.facing_tolerance)
        ):
            angular = 0.0
        angular = clamp(angular, -self.max_angular, self.max_angular)

        lateral = 0.0
        if self.use_lateral_alignment and alignment_weight > 0.0:
            # Optical +x is image-right; REP-103 base +y is robot-left.
            lateral = clamp(
                -alignment_weight * self.kp_lateral * x,
                -self.max_lateral,
                self.max_lateral,
            )
            if abs(bearing) <= self.angle_tolerance:
                lateral = 0.0

        linear = self.kp_linear * distance_error
        if abs(distance_error) <= self.distance_tolerance:
            linear = 0.0
        if abs(bearing) >= self.rotate_in_place_angle:
            linear = 0.0
        if not self.allow_reverse:
            linear = max(0.0, linear)
        linear = clamp(linear, -self.max_linear, self.max_linear)

        inside_goal = (
            abs(distance_error) <= self.distance_tolerance
            and abs(bearing) <= self.angle_tolerance
            and (
                not self.use_orientation_alignment
                or abs(facing_error) <= self.facing_tolerance
            )
        )
        inside_hold_zone = (
            abs(distance_error) <= self.distance_tolerance + self.distance_hysteresis
            and abs(bearing) <= self.angle_tolerance + self.angle_hysteresis
            and (
                not self.use_orientation_alignment
                or abs(facing_error)
                <= self.facing_tolerance + self.facing_hysteresis
            )
        )
        if inside_goal and self._arrival_started is None:
            self._arrival_started = now
            rospy.loginfo("Tag goal reached; holding position before StandDown")

        if self._arrival_started is not None and inside_hold_zone:
            # Stop inside a wider hold zone so pose-estimation noise does not
            # repeatedly drive the robot across the arrival threshold.
            linear = 0.0
            lateral = 0.0
            angular = 0.0
            if (now - self._arrival_started).to_sec() >= self.arrival_hold_time:
                self._finish_approach()
                return
        elif self._arrival_started is not None:
            rospy.logwarn("Tag left arrival hold zone; resuming visual servoing")
            self._arrival_started = None

        if inside_goal:
            linear = 0.0
            lateral = 0.0
            angular = 0.0

        command = Twist()
        command.linear.x = linear
        command.linear.y = lateral
        command.angular.z = angular
        self._publisher.publish(command)
        self._stopped = linear == 0.0 and lateral == 0.0 and angular == 0.0

    def _finish_approach(self):
        self.stop()
        if self.auto_lie_down:
            self._posture_publisher.publish("lie_down")
            rospy.loginfo("Tag approach complete: publishing Unitree-style StandDown")
        with self._lock:
            self._completed = True

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
