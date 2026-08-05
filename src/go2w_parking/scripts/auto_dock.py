#!/usr/bin/env python3
"""Mapless AprilTag + lidar visual-servo docking controller for Go2-W."""

import math
import threading

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class AutoDock:
    SEARCH = "SEARCH_TAG"
    LIDAR_SEEK = "LIDAR_GARAGE_SEEK"
    PRE_ALIGN = "PRE_ALIGN_OUTSIDE"
    APPROACH = "APPROACH"
    ENTER = "LIDAR_CENTERED_ENTRY"
    ALIGN = "FINE_ALIGN"
    HOLD = "HOLD"
    PARKED = "PARKED"
    LOST = "TAG_LOST"

    def __init__(self):
        get = rospy.get_param
        self.target_id = int(get("~target_id", 0))
        self.target_distance = get("~target_distance", 0.60)
        self.approach_distance = get("~approach_distance", 1.15)
        self.fine_align_distance = get("~fine_align_distance", 0.82)
        self.kp_forward = get("~kp_forward", 0.65)
        self.kp_lateral = get("~kp_lateral", 0.95)
        self.kp_bearing = get("~kp_bearing", 1.5)
        self.kp_facing = get("~kp_facing", 1.2)
        self.max_approach = get("~max_approach_speed", 0.52)
        self.max_final = get("~max_final_speed", 0.16)
        self.max_lateral = get("~max_lateral_speed", 0.24)
        self.max_angular = get("~max_angular_speed", 0.75)
        self.search_angular = get("~search_angular_speed", 0.25)
        self.lidar_seek_speed = get("~lidar_seek_speed", 0.22)
        self.lidar_reacquire_stop = get("~lidar_reacquire_stop_distance", 0.72)
        self.prealign_lateral_tol = get("~prealign_lateral_tolerance", 0.06)
        self.prealign_facing_tol = math.radians(
            get("~prealign_facing_tolerance_deg", 6.0)
        )
        self.kp_corridor_center = get("~kp_corridor_center", 0.65)
        self.kp_corridor_angle = get("~kp_corridor_angle", 1.0)
        self.corridor_max_width = get("~corridor_max_width", 1.6)
        self.position_tolerance = get("~position_tolerance", 0.035)
        self.lateral_tolerance = get("~lateral_tolerance", 0.025)
        self.facing_tolerance = math.radians(get("~facing_tolerance_deg", 5.0))
        self.hold_time = get("~hold_time", 1.0)
        self.lost_timeout = get("~lost_timeout", 0.35)
        self.alpha = clamp(get("~filter_alpha", 0.30), 0.0, 1.0)
        self.front_stop = get("~front_stop_distance", 0.26)
        self.side_stop = get("~side_stop_distance", 0.16)
        self.front_half_angle = math.radians(get("~scan_front_half_angle_deg", 22.0))
        self.side_half_angle = math.radians(get("~scan_side_half_angle_deg", 18.0))
        self.auto_lie_down = get("~auto_lie_down", True)
        self.control_rate = get("~control_rate", 20.0)
        self.trajectory_horizon = get("~trajectory_horizon", 3.0)
        self.trajectory_step = get("~trajectory_step", 0.10)

        self.lock = threading.Lock()
        self.last_seen = None
        self.tag_x = self.tag_z = self.facing = 0.0
        self.scan = None
        self.state = self.SEARCH
        self.hold_started = None
        self.completed = False

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/auto_dock/state", String, queue_size=1, latch=True)
        self.success_pub = rospy.Publisher("/auto_dock/success", Bool, queue_size=1, latch=True)
        self.posture_pub = rospy.Publisher("/go2w/posture_cmd", String, queue_size=1, latch=True)
        self.trajectory_pub = rospy.Publisher(
            "/auto_dock/predicted_path", Path, queue_size=1
        )
        self.lidar_lock_pub = rospy.Publisher(
            "/auto_dock/lidar_garage_detected", Bool, queue_size=1, latch=True
        )
        self.lidar_center_pub = rospy.Publisher(
            "/auto_dock/lidar_center_error", Float32, queue_size=1
        )
        self.lidar_heading_pub = rospy.Publisher(
            "/auto_dock/lidar_heading_error", Float32, queue_size=1
        )
        self.lidar_marker_pub = rospy.Publisher(
            "/auto_dock/lidar_markers", MarkerArray, queue_size=1
        )
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control)
        rospy.on_shutdown(self.stop)
        self.set_state(self.SEARCH)
        self.success_pub.publish(False)
        self.lidar_lock_pub.publish(False)
        rospy.loginfo("Mapless auto dock ready: tag=%d, target distance=%.3f m", self.target_id, self.target_distance)

    @staticmethod
    def facing_from_quaternion(q):
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-9:
            return None
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        normal_x = 2.0 * (x*z + y*w)
        normal_z = 1.0 - 2.0 * (x*x + y*y)
        if normal_z > 0.0:
            normal_x, normal_z = -normal_x, -normal_z
        return math.atan2(normal_x, -normal_z)

    def tag_callback(self, message):
        candidates = []
        for detection in message.detections:
            if self.target_id not in list(detection.id):
                continue
            pose = detection.pose.pose.pose
            facing = self.facing_from_quaternion(pose.orientation)
            if pose.position.z > 0.0 and facing is not None:
                candidates.append((pose.position.z, pose.position.x, facing))
        if not candidates:
            return
        z, x, facing = min(candidates)
        now = rospy.Time.now()
        with self.lock:
            if self.last_seen is None:
                self.tag_x, self.tag_z, self.facing = x, z, facing
            else:
                self.tag_x = self.alpha*x + (1.0-self.alpha)*self.tag_x
                self.tag_z = self.alpha*z + (1.0-self.alpha)*self.tag_z
                self.facing = wrap_angle(self.facing + self.alpha*wrap_angle(facing-self.facing))
            self.last_seen = now

    def scan_callback(self, message):
        with self.lock:
            self.scan = message

    def sector_min(self, center, half_width):
        scan = self.scan
        if scan is None or not scan.ranges:
            return float("inf")
        minimum = float("inf")
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(wrap_angle(angle - center)) <= half_width and math.isfinite(distance):
                if scan.range_min <= distance <= scan.range_max:
                    minimum = min(minimum, distance)
        return minimum

    @staticmethod
    def fit_line(points):
        if len(points) < 5:
            return None
        mean_x = sum(point[0] for point in points) / len(points)
        mean_y = sum(point[1] for point in points) / len(points)
        denominator = sum((point[0] - mean_x) ** 2 for point in points)
        if denominator < 1e-5:
            return None
        slope = sum((point[0] - mean_x) * (point[1] - mean_y)
                    for point in points) / denominator
        intercept = mean_y - slope * mean_x
        return slope, intercept

    def corridor_geometry(self):
        """Estimate kennel center and heading from Mid-360 wall returns.

        The fit is performed directly in base_link. It uses no map or odometry.
        """
        scan = self.scan
        if scan is None:
            return None
        left, right = [], []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not (scan.range_min <= distance <= scan.range_max):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            px, py = distance * math.cos(angle), distance * math.sin(angle)
            if 0.15 <= px <= 2.4 and 0.18 <= abs(py) <= self.corridor_max_width:
                (left if py > 0.0 else right).append((px, py))
        left_line, right_line = self.fit_line(left), self.fit_line(right)
        if left_line is None or right_line is None:
            return None
        left_slope, left_intercept = left_line
        right_slope, right_intercept = right_line
        width = left_intercept - right_intercept
        if not (0.5 <= width <= self.corridor_max_width):
            return None
        center_error = 0.5 * (left_intercept + right_intercept)
        wall_angle = math.atan(0.5 * (left_slope + right_slope))
        return center_error, wall_angle, width

    def publish_lidar_estimate(self, corridor):
        detected = corridor is not None
        self.lidar_lock_pub.publish(Bool(detected))
        markers = MarkerArray()
        if not detected:
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker.DELETEALL
            markers.markers.append(marker)
            self.lidar_marker_pub.publish(markers)
            return
        center, heading, width = corridor
        self.lidar_center_pub.publish(Float32(center))
        self.lidar_heading_pub.publish(Float32(heading))
        line = Marker()
        line.header.frame_id = "base_link"
        line.header.stamp = rospy.Time.now()
        line.ns, line.id = "mid360_garage", 0
        line.type, line.action = Marker.LINE_STRIP, Marker.ADD
        line.scale.x = 0.045
        line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 1.0, 0.25, 1.0
        for distance in (0.0, 2.2):
            point = Point()
            point.x = distance * math.cos(heading)
            point.y = center + distance * math.sin(heading)
            line.points.append(point)
        markers.markers.append(line)
        text = Marker()
        text.header = line.header
        text.ns, text.id = "mid360_garage", 1
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 1.0, center, 0.20
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color.r, text.color.g, text.color.b, text.color.a = 0.1, 0.8, 0.2, 1.0
        text.text = "MID360 GARAGE LOCK  width=%.2fm" % width
        markers.markers.append(text)
        self.lidar_marker_pub.publish(markers)

    def apply_lidar_safety(self, command):
        front = self.sector_min(0.0, self.front_half_angle)
        left = self.sector_min(math.pi/2.0, self.side_half_angle)
        right = self.sector_min(-math.pi/2.0, self.side_half_angle)
        if command.linear.x > 0.0 and front < self.front_stop:
            command.linear.x = 0.0
            rospy.logwarn_throttle(1.0, "Auto dock: front lidar safety stop")
        if command.linear.y > 0.0 and left < self.side_stop:
            command.linear.y = 0.0
        if command.linear.y < 0.0 and right < self.side_stop:
            command.linear.y = 0.0
        return command

    def publish_command(self, command):
        safe_command = self.apply_lidar_safety(command)
        self.cmd_pub.publish(safe_command)
        self.publish_trajectory(safe_command)

    def publish_trajectory(self, command):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "base_link"
        x = y = yaw = 0.0
        steps = max(1, int(self.trajectory_horizon / self.trajectory_step))
        for index in range(steps + 1):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * command.linear.x - s * command.linear.y) * self.trajectory_step
            y += (s * command.linear.x + c * command.linear.y) * self.trajectory_step
            yaw = wrap_angle(yaw + command.angular.z * self.trajectory_step)
        self.trajectory_pub.publish(path)

    def lidar_seek_command(self, corridor):
        center, heading, _width = corridor
        command = Twist()
        front = self.sector_min(0.0, self.front_half_angle)
        if front > self.lidar_reacquire_stop and abs(heading) < math.radians(28.0):
            command.linear.x = self.lidar_seek_speed
        command.linear.y = clamp(self.kp_corridor_center * center,
                                 -self.max_lateral, self.max_lateral)
        command.angular.z = clamp(self.kp_corridor_angle * heading,
                                  -self.max_angular, self.max_angular)
        return command

    def control(self, _event):
        now = rospy.Time.now()
        with self.lock:
            last_seen, x, z, facing = self.last_seen, self.tag_x, self.tag_z, self.facing
            completed = self.completed
        if completed:
            self.stop()
            return

        corridor = self.corridor_geometry()
        self.publish_lidar_estimate(corridor)

        if last_seen is None:
            if corridor is not None:
                self.set_state(self.LIDAR_SEEK)
                command = self.lidar_seek_command(corridor)
            else:
                self.set_state(self.SEARCH)
                command = Twist()
                command.angular.z = self.search_angular
            self.publish_command(command)
            return
        if (now - last_seen).to_sec() > self.lost_timeout:
            self.hold_started = None
            if corridor is not None:
                self.set_state(self.LIDAR_SEEK)
                self.publish_command(self.lidar_seek_command(corridor))
            else:
                self.set_state(self.LOST)
                self.stop()
            return

        distance_error = z - self.target_distance
        bearing = math.atan2(x, z)
        command = Twist()

        corridor_center = corridor[0] if corridor is not None else 0.0
        corridor_angle = corridor[1] if corridor is not None else 0.0

        if z > self.approach_distance:
            outside_aligned = (
                abs(x) <= self.prealign_lateral_tol
                and abs(facing) <= self.prealign_facing_tol
            )
            if not outside_aligned:
                self.set_state(self.PRE_ALIGN)
                # Explicitly prohibit entry until lateral and heading errors
                # are corrected outside the kennel.
                command.linear.x = 0.0
                command.linear.y = clamp(-self.kp_lateral * x,
                                         -self.max_lateral, self.max_lateral)
                command.angular.z = clamp(
                    self.kp_facing * facing - 0.35 * self.kp_bearing * bearing,
                    -self.max_angular, self.max_angular)
            else:
                self.set_state(self.APPROACH)
                command.linear.x = clamp(self.kp_forward * distance_error,
                                         0.0, self.max_approach)
                command.linear.y = clamp(self.kp_corridor_center * corridor_center,
                                         -0.12, 0.12)
                command.angular.z = clamp(
                    -self.kp_bearing * bearing + self.kp_corridor_angle * corridor_angle,
                    -self.max_angular, self.max_angular)
        else:
            self.set_state(self.ENTER if z > self.fine_align_distance else self.ALIGN)
            speed_limit = self.max_final if z <= self.fine_align_distance else self.max_approach
            command.linear.x = clamp(self.kp_forward * distance_error,
                                     -self.max_final, speed_limit)
            command.linear.y = clamp(-self.kp_lateral * x
                                     + self.kp_corridor_center * corridor_center,
                                     -self.max_lateral, self.max_lateral)
            command.angular.z = clamp(
                -0.55*self.kp_bearing*bearing + self.kp_facing*facing
                + self.kp_corridor_angle*corridor_angle,
                -self.max_angular, self.max_angular)

        inside = (abs(distance_error) <= self.position_tolerance
                  and abs(x) <= self.lateral_tolerance
                  and abs(facing) <= self.facing_tolerance)
        if inside:
            command = Twist()
            self.set_state(self.HOLD)
            if self.hold_started is None:
                self.hold_started = now
            elif (now - self.hold_started).to_sec() >= self.hold_time:
                self.finish()
                return
        else:
            self.hold_started = None

        self.publish_command(command)

    def set_state(self, state):
        if state != self.state:
            rospy.loginfo("Auto dock state: %s -> %s", self.state, state)
            self.state = state
        self.state_pub.publish(String(self.state))

    def finish(self):
        self.stop()
        self.set_state(self.PARKED)
        self.success_pub.publish(True)
        if self.auto_lie_down:
            self.posture_pub.publish(String("lie_down"))
        with self.lock:
            self.completed = True
        rospy.loginfo("Auto dock complete; charging pose reached")

    def stop(self):
        command = Twist()
        self.cmd_pub.publish(command)
        self.publish_trajectory(command)


if __name__ == "__main__":
    rospy.init_node("auto_dock")
    try:
        AutoDock()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logerr("auto_dock stopped: %s", error)
