#!/usr/bin/env python3
"""Mapless AprilTag + lidar visual-servo docking controller for Go2-W.

Docking starts at PREALIGN_GATE (no search / lidar-seek beforehand):
  1) PREALIGN_GATE  – face Tag normal, strafe to centerline, hold front lidar 1.5±0.05 m
  2) ENTER_BAY      – camera+lidar straight entry; near pad switch to lidar-only
  3) CROUCH         – stop and send lie_down / squat posture
"""

import math
import threading

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class AutoDock:
    WAIT_TAG = "WAIT_TAG"
    PREALIGN_GATE = "PREALIGN_GATE"
    ENTER_BAY = "ENTER_BAY"
    CROUCH = "CROUCH"
    PARKED = "PARKED"
    LOST = "TAG_LOST"

    def __init__(self):
        get = rospy.get_param
        self.target_id = int(get("~target_id", 0))
        # Stage-1 gate: front lidar hold distance.
        self.gate_front_distance = get("~gate_front_distance", 1.50)
        self.gate_front_tolerance = get("~gate_front_tolerance", 0.05)
        self.gate_facing_tol = math.radians(get("~gate_facing_tolerance_deg", 3.0))
        self.gate_lateral_tol = get("~gate_lateral_tolerance", 0.06)
        self.gate_hold_time = get("~gate_hold_time", 2.0)
        # Stage-2 / 3: lidar final stop in front of Tag / back wall.
        self.target_distance = get("~target_distance", 0.35)
        self.final_front_tolerance = get("~final_front_tolerance", 0.05)
        self.final_lateral_tolerance = get("~final_lateral_tolerance", 0.05)
        # When Tag optical y/z exceeds this, treat Tag as at bottom of image.
        self.tag_bottom_yx_ratio = get("~tag_bottom_yx_ratio", 0.22)
        self.tag_bottom_range = get("~tag_bottom_range", 0.55)

        self.kp_forward = get("~kp_forward", 0.55)
        self.kp_lateral = get("~kp_lateral", 0.95)
        self.kp_facing = get("~kp_facing", 1.2)
        self.max_approach = get("~max_approach_speed", 0.38)
        self.max_final = get("~max_final_speed", 0.12)
        self.max_lateral = get("~max_lateral_speed", 0.18)
        self.max_angular = get("~max_angular_speed", 0.60)
        self.kp_corridor_center = get("~kp_corridor_center", 0.65)
        self.kp_corridor_angle = get("~kp_corridor_angle", 1.0)
        self.corridor_max_width = get("~corridor_max_width", 1.6)
        self.lost_timeout = get("~lost_timeout", 0.35)
        # Longer grace while squaring up so brief Tag dropouts do not chatter TAG_LOST.
        self.prealign_lost_timeout = get("~prealign_lost_timeout", 2.0)
        self.alpha = clamp(get("~filter_alpha", 0.30), 0.0, 1.0)
        self.front_stop = get("~front_stop_distance", 0.22)
        self.side_stop = get("~side_stop_distance", 0.16)
        self.front_half_angle = math.radians(get("~scan_front_half_angle_deg", 22.0))
        self.side_half_angle = math.radians(get("~scan_side_half_angle_deg", 18.0))
        self.auto_lie_down = get("~auto_lie_down", True)
        self.auto_reset_after_park = get("~auto_reset_after_park", True)
        self.reset_delay = get("~reset_delay", 1.5)
        self.restart_after_reset = get("~restart_after_reset", True)
        self.control_rate = get("~control_rate", 20.0)
        self.trajectory_horizon = get("~trajectory_horizon", 3.0)
        self.trajectory_step = get("~trajectory_step", 0.10)

        self.lock = threading.Lock()
        self.last_seen = None
        self.tag_x = self.tag_y = self.tag_z = self.facing = 0.0
        self.scan = None
        self.state = self.WAIT_TAG
        self.gate_done = False
        self.gate_ok_since = None
        self.lidar_only = False
        self.completed = False
        self.reset_pending = False
        self.enabled = rospy.get_param("~start_enabled", True)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub = rospy.Publisher("/auto_dock/state", String, queue_size=1, latch=True)
        self.success_pub = rospy.Publisher("/auto_dock/success", Bool, queue_size=1, latch=True)
        self.posture_pub = rospy.Publisher("/go2w/posture_cmd", String, queue_size=1, latch=True)
        self.reset_pub = rospy.Publisher("/go2w/sim_reset", String, queue_size=1)
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
        rospy.Subscriber("/control_mode", String, self.mode_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control)
        rospy.on_shutdown(self.stop)
        self.set_state(self.WAIT_TAG)
        self.success_pub.publish(False)
        self.lidar_lock_pub.publish(False)
        rospy.loginfo(
            "Mapless auto dock ready: tag=%d, gate=%.2fm, final=%.2fm, enabled=%s",
            self.target_id, self.gate_front_distance, self.target_distance, self.enabled,
        )

    def reset_cycle_flags(self):
        self.gate_done = False
        self.gate_ok_since = None
        self.lidar_only = False
        self.completed = False

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
                candidates.append(
                    (pose.position.z, pose.position.x, pose.position.y, facing)
                )
        if not candidates:
            return
        z, x, y, facing = min(candidates)
        now = rospy.Time.now()
        with self.lock:
            if self.last_seen is None:
                self.tag_x, self.tag_y, self.tag_z, self.facing = x, y, z, facing
            else:
                self.tag_x = self.alpha*x + (1.0-self.alpha)*self.tag_x
                self.tag_y = self.alpha*y + (1.0-self.alpha)*self.tag_y
                self.tag_z = self.alpha*z + (1.0-self.alpha)*self.tag_z
                self.facing = wrap_angle(
                    self.facing + self.alpha*wrap_angle(facing-self.facing)
                )
            self.last_seen = now

    def scan_callback(self, message):
        with self.lock:
            self.scan = message

    def mode_callback(self, message):
        mode = message.data.strip().lower()
        enable = mode == "auto"
        with self.lock:
            was_enabled = self.enabled
            self.enabled = enable
            if enable and not was_enabled:
                self.reset_cycle_flags()
                self.state = self.WAIT_TAG
        if enable and not was_enabled:
            self.set_state(self.WAIT_TAG)
            self.success_pub.publish(False)
            rospy.loginfo("Auto dock enabled by /control_mode")
        elif (not enable) and was_enabled:
            self.stop()
            rospy.loginfo("Auto dock paused; manual teleop owns /cmd_vel")

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
        """Estimate kennel center and heading from Mid-360 wall returns."""
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
        for _ in range(steps + 1):
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

    def orientation_aligned(self, x, facing):
        """Aligned when Tag is centered laterally and body faces Tag normal."""
        return (
            abs(facing) <= self.gate_facing_tol
            and abs(x) <= self.gate_lateral_tol
        )

    def tag_at_image_bottom(self, y, z):
        """Tag dropped to the lower part of the camera frame (optical y down)."""
        if z <= self.tag_bottom_range:
            return True
        if z <= 1e-3:
            return False
        return (y / z) >= self.tag_bottom_yx_ratio

    def prealign_gate_command(self, x, z, facing, front):
        """Stage 1: strafe to Tag centerline + face Tag normal, then hold 1.5 m.

        Bearing is not used — facing + lateral x alone put the dog on the
        Tag frontal axis without yaw fighting.
        """
        del z
        command = Twist()
        command.linear.y = clamp(-self.kp_lateral * x,
                                 -self.max_lateral, self.max_lateral)
        command.angular.z = clamp(self.kp_facing * facing,
                                  -self.max_angular, self.max_angular)

        aligned = self.orientation_aligned(x, facing)
        if aligned and math.isfinite(front):
            range_error = front - self.gate_front_distance
            command.linear.x = clamp(
                self.kp_forward * range_error, -self.max_final, self.max_approach)
        else:
            command.linear.x = 0.0
        return command

    def enter_bay_command(self, x, z, facing, corridor, front):
        """Stage 2a: camera + lidar keep a straight entry toward the pad."""
        command = Twist()
        center = corridor[0] if corridor is not None else 0.0
        heading = corridor[1] if corridor is not None else 0.0
        if math.isfinite(front) and front < 8.0:
            range_error = front - self.target_distance
        else:
            range_error = z - self.target_distance
        speed_limit = self.max_final if front < 0.70 else self.max_approach
        command.linear.x = clamp(self.kp_forward * range_error,
                                 -self.max_final, speed_limit)
        command.linear.y = clamp(
            -self.kp_lateral * x + self.kp_corridor_center * center,
            -self.max_lateral, self.max_lateral)
        command.angular.z = clamp(
            self.kp_facing * facing + self.kp_corridor_angle * heading,
            -self.max_angular, self.max_angular)
        return command

    def lidar_final_command(self, corridor, front):
        """Stage 2b: Tag at image bottom / unreliable — lidar-only final pose."""
        command = Twist()
        center = corridor[0] if corridor is not None else 0.0
        heading = corridor[1] if corridor is not None else 0.0
        if math.isfinite(front):
            command.linear.x = clamp(
                self.kp_forward * (front - self.target_distance),
                -self.max_final, self.max_final)
        command.linear.y = clamp(self.kp_corridor_center * center,
                                 -self.max_lateral, self.max_lateral)
        command.angular.z = clamp(self.kp_corridor_angle * heading,
                                  -self.max_angular, self.max_angular)
        return command

    def control(self, _event):
        now = rospy.Time.now()
        with self.lock:
            last_seen = self.last_seen
            x, y, z, facing = self.tag_x, self.tag_y, self.tag_z, self.facing
            completed = self.completed
            enabled = self.enabled
            gate_done = self.gate_done
            lidar_only = self.lidar_only
        corridor = self.corridor_geometry()
        self.publish_lidar_estimate(corridor)
        front = self.sector_min(0.0, self.front_half_angle)
        if not enabled:
            return
        if completed:
            self.stop()
            return

        lose_after = self.lost_timeout if gate_done else self.prealign_lost_timeout
        tag_fresh = (
            last_seen is not None
            and (now - last_seen).to_sec() <= lose_after
        )

        # --- No Tag / lost Tag: no search before pre-align ---
        if not tag_fresh:
            if not gate_done:
                self.gate_ok_since = None
                if last_seen is None:
                    self.set_state(self.WAIT_TAG)
                else:
                    self.set_state(self.LOST)
                self.stop()
                return
            if corridor is not None:
                self.lidar_only = True
                self.set_state(self.ENTER_BAY)
                command = self.lidar_final_command(corridor, front)
                if self.lidar_final_ready(corridor, front):
                    self.finish_crouch()
                    return
                self.publish_command(command)
                return
            self.set_state(self.LOST)
            self.stop()
            return

        # ========== Stage 1: PREALIGN_GATE (first active stage) ==========
        if not gate_done:
            self.set_state(self.PREALIGN_GATE)
            command = self.prealign_gate_command(x, z, facing, front)
            oriented = self.orientation_aligned(x, facing)
            ranged = (
                math.isfinite(front)
                and abs(front - self.gate_front_distance) <= self.gate_front_tolerance
            )
            if oriented and ranged:
                if self.gate_ok_since is None:
                    self.gate_ok_since = now
                    rospy.loginfo(
                        "Gate conditions met; holding %.1fs (facing=%.1fdeg x=%.3fm front=%.2fm)",
                        self.gate_hold_time, math.degrees(facing), x, front,
                    )
                command = Twist()
                held = (now - self.gate_ok_since).to_sec() >= self.gate_hold_time
                if held:
                    self.gate_done = True
                    self.gate_ok_since = None
                    rospy.loginfo(
                        "Pre-align gate stable for %.1fs; entering stage 2",
                        self.gate_hold_time,
                    )
                    self.set_state(self.ENTER_BAY)
            else:
                self.gate_ok_since = None
            self.publish_command(command)
            return

        # ========== Stage 2: ENTER_BAY (held until stage 3) ==========
        if (not lidar_only) and self.tag_at_image_bottom(y, z):
            self.lidar_only = True
            rospy.loginfo(
                "Tag at image bottom (y/z=%.3f, z=%.2f); lidar-only within stage 2",
                (y / z) if z > 1e-3 else 0.0, z,
            )

        self.set_state(self.ENTER_BAY)
        if self.lidar_only:
            command = self.lidar_final_command(corridor, front)
        else:
            command = self.enter_bay_command(x, z, facing, corridor, front)
        if self.lidar_final_ready(corridor, front):
            self.finish_crouch()
            return
        self.publish_command(command)

    def lidar_final_ready(self, corridor, front):
        if not math.isfinite(front):
            return False
        if abs(front - self.target_distance) > self.final_front_tolerance:
            return False
        center = corridor[0] if corridor is not None else float("inf")
        return abs(center) <= self.final_lateral_tolerance

    def set_state(self, state):
        if state != self.state:
            rospy.loginfo("Auto dock state: %s -> %s", self.state, state)
            self.state = state
        self.state_pub.publish(String(self.state))

    def finish_crouch(self):
        """Stage 3: stop and squat / lie down on the charging pad."""
        self.stop()
        self.set_state(self.CROUCH)
        self.success_pub.publish(True)
        if self.auto_lie_down:
            self.posture_pub.publish(String(data="lie_down"))
            rospy.loginfo("Stage 3: crouch / lie_down commanded")
        with self.lock:
            self.completed = True
            already_pending = self.reset_pending
            if self.auto_reset_after_park:
                self.reset_pending = True
        self.set_state(self.PARKED)
        rospy.loginfo("Auto dock complete; charging pose reached")
        if self.auto_reset_after_park and not already_pending:
            rospy.Timer(
                rospy.Duration(self.reset_delay),
                self.reset_after_park,
                oneshot=True,
            )

    def reset_after_park(self, _event):
        try:
            rospy.wait_for_service("/parking/reset", timeout=0.3)
            rospy.ServiceProxy("/parking/reset", Empty)()
            rospy.loginfo("Called /parking/reset")
        except Exception:
            pass
        self.reset_pub.publish(String(data="reset"))
        self.posture_pub.publish(String(data="stand_up"))
        self.stop()
        with self.lock:
            self.reset_cycle_flags()
            self.last_seen = None
            self.tag_x = self.tag_y = self.tag_z = self.facing = 0.0
            self.reset_pending = False
            if self.restart_after_reset:
                self.state = self.WAIT_TAG
        self.success_pub.publish(False)
        if self.restart_after_reset:
            self.set_state(self.WAIT_TAG)
            rospy.loginfo("Reset to initial pose; waiting for Tag / pre-align")
        else:
            self.set_state(self.PARKED)
            rospy.loginfo("Reset to initial pose; waiting for next auto cycle")

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
