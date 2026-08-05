#!/usr/bin/env python3
"""Lightweight 2-D holonomic parking simulator for a Go2-W-sized robot."""

import math
import random
import threading

import rospy
import tf
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def approach(value, target, max_step):
    return value + clamp(target - value, -max_step, max_step)


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class ParkingSim:
    def __init__(self):
        get = rospy.get_param
        self.length = get("~robot_length", 1.20)
        self.width = get("~robot_width", 0.80)
        self.max_forward = get("~max_forward", 0.80)
        self.max_reverse = get("~max_reverse", 0.45)
        self.max_lateral = get("~max_lateral", 0.40)
        self.max_angular = get("~max_angular", 1.20)
        self.accel_forward = get("~accel_forward", 0.90)
        self.accel_lateral = get("~accel_lateral", 0.70)
        self.accel_angular = get("~accel_angular", 2.00)
        self.timeout = get("~command_timeout", 0.40)

        self.spawn_center_x = get("~spawn_region_center_x", -2.0)
        self.spawn_center_y = get("~spawn_region_center_y", 0.0)
        self.spawn_region_size = get("~spawn_region_size", 2.0)
        self.spawn_yaw_random = bool(get("~spawn_yaw_random", True))
        self.initial = (
            get("~initial_x", self.spawn_center_x),
            get("~initial_y", self.spawn_center_y),
            get("~initial_yaw", 0.0),
        )
        self.back_x = get("~kennel_back_x", 1.29)
        self.front_x = get("~kennel_front_x", 0.0)
        self.half_width = get("~kennel_half_width", 0.50)
        self.wall_thickness = get("~wall_thickness", 0.08)
        self.pad_x = get("~charger_pad_x", 0.70)
        self.pad_y = get("~charger_pad_y", 0.0)
        self.pad_length = get("~charger_pad_length", 1.20)
        self.pad_width = get("~charger_pad_width", 0.70)
        self.bridge_length = get("~charger_bridge_length", 0.06)
        self.bridge_width = get("~charger_bridge_width", 0.40)
        self.goal = (
            get("~goal_x", 0.60),
            get("~goal_y", 0.0),
            get("~goal_yaw", 0.0),
        )
        self.goal_pos_tol = get("~goal_position_tolerance", 0.10)
        self.goal_yaw_tol = math.radians(get("~goal_yaw_tolerance_deg", 8.0))
        self.goal_speed_tol = get("~goal_speed_tolerance", 0.03)
        self.goal_hold = get("~goal_hold_time", 1.0)

        self.map_res = get("~map_resolution", 0.04)
        self.map_min_x = get("~map_min_x", -3.0)
        self.map_max_x = get("~map_max_x", 2.4)
        self.map_min_y = get("~map_min_y", -2.0)
        self.map_max_y = get("~map_max_y", 2.0)
        self.scan_beams = int(get("~scan_beams", 180))
        self.scan_range = get("~scan_range", 4.0)
        self.sim_rate = get("~simulation_rate", 50.0)
        self.publish_rate = get("~publish_rate", 20.0)
        self.publish_tag = get("~publish_tag_detection", True)
        self.tag_id = int(get("~tag_id", 0))
        self.tag_size = get("~tag_size", 0.11)
        self.tag_x = get("~tag_x", 1.25)
        self.tag_y = get("~tag_y", 0.0)
        self.tag_max_range = get("~tag_max_range", 4.0)
        self.tag_half_fov = math.radians(get("~tag_horizontal_fov_deg", 80.0) / 2.0)
        self.camera_width = int(get("~camera_width", 424))
        self.camera_height = int(get("~camera_height", 240))
        self.camera_fx = get("~camera_fx", 310.0)
        self.camera_fy = get("~camera_fy", 310.0)

        if min(self.length, self.width, self.map_res, self.sim_rate) <= 0.0:
            raise ValueError("robot size, map resolution and simulation rate must be positive")

        # Axis-aligned wall rectangles: center x/y and full x/y size.
        span = self.back_x - self.front_x
        self.walls = [
            (self.back_x, 0.0, self.wall_thickness, 2.0 * self.half_width + self.wall_thickness),
            ((self.front_x + self.back_x) / 2.0, self.half_width, span, self.wall_thickness),
            ((self.front_x + self.back_x) / 2.0, -self.half_width, span, self.wall_thickness),
        ]

        self.lock = threading.Lock()
        self.x, self.y, self.yaw = self.sample_spawn_pose()
        self.vx = self.vy = self.wz = 0.0
        self.cmd = Twist()
        self.last_cmd = rospy.Time(0)
        self.last_update = rospy.Time.now()
        self.last_publish = rospy.Time(0)
        self.hold_started = None
        self.parked = False
        self.collision = False
        self.path = Path()
        self.path.header.frame_id = "map"

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=5)
        self.map_pub = rospy.Publisher("/parking/map", OccupancyGrid, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/parking/markers", MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher("/parking/path", Path, queue_size=1)
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.goal_pub = rospy.Publisher("/parking/goal", PoseStamped, queue_size=1, latch=True)
        self.parked_pub = rospy.Publisher("/parking/parked", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/parking/status", String, queue_size=1, latch=True)
        self.tag_pub = rospy.Publisher("/tag_detections", AprilTagDetectionArray, queue_size=1)
        self.image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher(
            "/camera/color/camera_info", CameraInfo, queue_size=1, latch=True
        )
        self.tf_pub = tf.TransformBroadcaster()

        rospy.Subscriber("/cmd_vel", Twist, self.command_callback, queue_size=1)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback, queue_size=1)
        rospy.Service("/parking/reset", Empty, self.reset_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.sim_rate), self.update)

        rospy.sleep(0.1)
        self.map_pub.publish(self.make_map())
        self.goal_pub.publish(self.make_goal())
        self.camera_info_pub.publish(self.make_camera_info(rospy.Time.now()))
        self.publish_state(rospy.Time.now())
        rospy.loginfo(
            "Go2-W parking sim ready: spawn region center=(%.2f, %.2f) size=%.2f, goal=(%.2f, %.2f)",
            self.spawn_center_x, self.spawn_center_y, self.spawn_region_size,
            self.goal[0], self.goal[1],
        )

    def sample_spawn_pose(self):
        """Uniform sample inside the configured square, optionally random yaw."""
        half = 0.5 * self.spawn_region_size
        for _ in range(40):
            x = self.spawn_center_x + random.uniform(-half, half)
            y = self.spawn_center_y + random.uniform(-half, half)
            if self.spawn_yaw_random:
                yaw = random.uniform(-math.pi, math.pi)
            else:
                yaw = self.initial[2]
            if not self.collides(x, y, yaw):
                return x, y, wrap_angle(yaw)
        rospy.logwarn("Spawn sampling fell back to region center")
        return self.spawn_center_x, self.spawn_center_y, self.initial[2]

    def command_callback(self, message):
        with self.lock:
            self.cmd = message
            self.last_cmd = rospy.Time.now()

    def initial_pose_callback(self, message):
        q = message.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        self.set_pose(message.pose.pose.position.x, message.pose.pose.position.y, yaw)

    def reset_callback(self, _request):
        self.set_pose(*self.sample_spawn_pose())
        return EmptyResponse()

    def set_pose(self, x, y, yaw):
        with self.lock:
            if self.collides(x, y, yaw):
                rospy.logwarn("Rejected reset/initial pose inside kennel wall")
                return
            self.x, self.y, self.yaw = x, y, wrap_angle(yaw)
            self.vx = self.vy = self.wz = 0.0
            self.cmd = Twist()
            self.last_cmd = rospy.Time(0)
            self.hold_started = None
            self.parked = False
            self.collision = False
            self.path.poses = []

    def update(self, event):
        now = event.current_real
        dt = clamp((now - self.last_update).to_sec(), 0.0, 0.10)
        self.last_update = now
        if dt <= 0.0:
            return

        with self.lock:
            active = (now - self.last_cmd).to_sec() <= self.timeout
            target_x = self.cmd.linear.x if active else 0.0
            target_y = self.cmd.linear.y if active else 0.0
            target_w = self.cmd.angular.z if active else 0.0
            target_x = clamp(target_x, -self.max_reverse, self.max_forward)
            target_y = clamp(target_y, -self.max_lateral, self.max_lateral)
            target_w = clamp(target_w, -self.max_angular, self.max_angular)
            self.vx = approach(self.vx, target_x, self.accel_forward * dt)
            self.vy = approach(self.vy, target_y, self.accel_lateral * dt)
            self.wz = approach(self.wz, target_w, self.accel_angular * dt)

            c, s = math.cos(self.yaw), math.sin(self.yaw)
            nx = self.x + (c * self.vx - s * self.vy) * dt
            ny = self.y + (s * self.vx + c * self.vy) * dt
            nyaw = wrap_angle(self.yaw + self.wz * dt)
            self.collision = self.collides(nx, ny, nyaw)
            if self.collision:
                self.vx = self.vy = self.wz = 0.0
            else:
                self.x, self.y, self.yaw = nx, ny, nyaw
            self.update_parking_state(now)

        if (now - self.last_publish).to_sec() >= 1.0 / self.publish_rate:
            self.last_publish = now
            self.publish_state(now)

    def robot_corners(self, x, y, yaw):
        hx, hy = self.length / 2.0, self.width / 2.0
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * px - s * py, y + s * px + c * py)
                for px, py in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]

    @staticmethod
    def projections(points, axis):
        values = [px * axis[0] + py * axis[1] for px, py in points]
        return min(values), max(values)

    def rectangle_overlap(self, robot, wall):
        wx, wy, sx, sy = wall
        other = [(wx - sx / 2, wy - sy / 2), (wx + sx / 2, wy - sy / 2),
                 (wx + sx / 2, wy + sy / 2), (wx - sx / 2, wy + sy / 2)]
        axes = [(1.0, 0.0), (0.0, 1.0)]
        for points in (robot, other):
            for index in range(2):
                edge = (points[index + 1][0] - points[index][0], points[index + 1][1] - points[index][1])
                norm = math.hypot(*edge)
                axes.append((-edge[1] / norm, edge[0] / norm))
        for axis in axes:
            a0, a1 = self.projections(robot, axis)
            b0, b1 = self.projections(other, axis)
            if a1 <= b0 or b1 <= a0:
                return False
        return True

    def collides(self, x, y, yaw):
        corners = self.robot_corners(x, y, yaw)
        if any(px < self.map_min_x or px > self.map_max_x or py < self.map_min_y or py > self.map_max_y for px, py in corners):
            return True
        return any(self.rectangle_overlap(corners, wall) for wall in self.walls)

    def point_in_wall(self, x, y):
        return any(abs(x - wx) <= sx / 2.0 and abs(y - wy) <= sy / 2.0 for wx, wy, sx, sy in self.walls)

    @staticmethod
    def segment_intersects_rectangle(x0, y0, x1, y1, wall, endpoint_margin=1e-3):
        """Return true when a wall blocks the open segment from camera to target.

        The slab test is deterministic and avoids sampling holes through thin
        walls.  Intersections at the target endpoint are ignored so a tag may
        be mounted directly on the front face of the kennel back wall.
        """
        wx, wy, sx, sy = wall
        lower = (wx - sx / 2.0, wy - sy / 2.0)
        upper = (wx + sx / 2.0, wy + sy / 2.0)
        direction = (x1 - x0, y1 - y0)
        start = (x0, y0)
        t_min, t_max = 0.0, 1.0
        for axis in range(2):
            if abs(direction[axis]) < 1e-12:
                if start[axis] < lower[axis] or start[axis] > upper[axis]:
                    return False
                continue
            inverse = 1.0 / direction[axis]
            enter = (lower[axis] - start[axis]) * inverse
            leave = (upper[axis] - start[axis]) * inverse
            if enter > leave:
                enter, leave = leave, enter
            t_min = max(t_min, enter)
            t_max = min(t_max, leave)
            if t_min > t_max:
                return False
        return t_max > endpoint_margin and t_min < 1.0 - endpoint_margin

    def tag_has_line_of_sight(self):
        return not any(self.segment_intersects_rectangle(
            self.x, self.y, self.tag_x, self.tag_y, wall) for wall in self.walls)

    def tag_is_visible(self, optical_z, distance, bearing):
        return (optical_z > 0.0 and distance <= self.tag_max_range
                and abs(bearing) <= self.tag_half_fov
                and self.tag_has_line_of_sight())

    def update_parking_state(self, now):
        gx, gy, gyaw = self.goal
        position_ok = math.hypot(self.x - gx, self.y - gy) <= self.goal_pos_tol
        yaw_ok = abs(wrap_angle(self.yaw - gyaw)) <= self.goal_yaw_tol
        speed_ok = max(abs(self.vx), abs(self.vy), abs(self.wz)) <= self.goal_speed_tol
        inside = position_ok and yaw_ok and speed_ok
        if inside:
            if self.hold_started is None:
                self.hold_started = now
            if (now - self.hold_started).to_sec() >= self.goal_hold:
                self.parked = True
        else:
            self.hold_started = None
            self.parked = False

    def make_map(self):
        width = int(math.ceil((self.map_max_x - self.map_min_x) / self.map_res))
        height = int(math.ceil((self.map_max_y - self.map_min_y) / self.map_res))
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.info.resolution = self.map_res
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = self.map_min_x
        message.info.origin.position.y = self.map_min_y
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (width * height)
        for iy in range(height):
            y = self.map_min_y + (iy + 0.5) * self.map_res
            for ix in range(width):
                x = self.map_min_x + (ix + 0.5) * self.map_res
                if self.point_in_wall(x, y):
                    message.data[iy * width + ix] = 100
        return message

    def make_goal(self):
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time.now()
        message.pose.position.x, message.pose.position.y = self.goal[:2]
        message.pose.orientation = self.quaternion(self.goal[2])
        return message

    @staticmethod
    def quaternion(yaw):
        values = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        from geometry_msgs.msg import Quaternion
        return Quaternion(*values)

    def raycast(self, angle):
        step = max(0.015, self.map_res / 2.0)
        distance = 0.06
        while distance < self.scan_range:
            px = self.x + distance * math.cos(angle)
            py = self.y + distance * math.sin(angle)
            if self.point_in_wall(px, py):
                return distance
            distance += step
        return float("inf")

    def publish_state(self, stamp):
        q = self.quaternion(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = self.vx, self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)
        self.tf_pub.sendTransform((0, 0, 0), (0, 0, 0, 1), stamp, "odom", "map")
        self.tf_pub.sendTransform((self.x, self.y, 0), (q.x, q.y, q.z, q.w), stamp, "base_link", "odom")

        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "map"
        pose.pose.position.x, pose.pose.position.y = self.x, self.y
        pose.pose.orientation = q
        if not self.path.poses or math.hypot(self.path.poses[-1].pose.position.x - self.x, self.path.poses[-1].pose.position.y - self.y) > 0.025:
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-2000:]
        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)
        self.marker_pub.publish(self.make_markers(stamp))
        self.scan_pub.publish(self.make_scan(stamp))
        if self.publish_tag:
            self.tag_pub.publish(self.make_tag_detection(stamp))
        self.image_pub.publish(self.make_camera_image(stamp))
        self.parked_pub.publish(Bool(self.parked))
        state = "PARKED" if self.parked else ("COLLISION" if self.collision else "MOVING")
        self.status_pub.publish(String(state))

    def make_scan(self, stamp):
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, "base_link"
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2.0 * math.pi / self.scan_beams
        scan.range_min, scan.range_max = 0.06, self.scan_range
        scan.ranges = [self.raycast(self.yaw + scan.angle_min + i * scan.angle_increment) for i in range(self.scan_beams)]
        return scan

    def make_tag_detection(self, stamp):
        message = AprilTagDetectionArray()
        message.header.stamp = stamp
        message.header.frame_id = "camera_color_optical_frame"
        dx, dy = self.tag_x - self.x, self.tag_y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        optical_x = s * dx - c * dy
        optical_z = c * dx + s * dy
        bearing = math.atan2(optical_x, optical_z)
        if not self.tag_is_visible(optical_z, math.hypot(dx, dy), bearing):
            return message

        detection = AprilTagDetection()
        detection.id = [self.tag_id]
        detection.size = [self.tag_size]
        detection.pose.header = message.header
        detection.pose.pose.pose.position.x = optical_x
        # Synthetic optical-down so near range looks like Tag dropping to image bottom.
        detection.pose.pose.pose.position.y = clamp(0.28 * (0.95 - optical_z), 0.0, 0.40)
        detection.pose.pose.pose.position.z = optical_z
        # Tag +Z normal faces out of the kennel (-map x). Express that normal
        # in the camera optical frame as a rotation about optical +Y.
        normal_angle = math.pi + self.yaw
        detection.pose.pose.pose.orientation.y = math.sin(normal_angle / 2.0)
        detection.pose.pose.pose.orientation.w = math.cos(normal_angle / 2.0)
        message.detections.append(detection)
        return message

    def make_camera_info(self, stamp):
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = "camera_color_optical_frame"
        message.width, message.height = self.camera_width, self.camera_height
        cx, cy = self.camera_width / 2.0, self.camera_height / 2.0
        message.K = [self.camera_fx, 0.0, cx,
                     0.0, self.camera_fy, cy,
                     0.0, 0.0, 1.0]
        message.P = [self.camera_fx, 0.0, cx, 0.0,
                     0.0, self.camera_fy, cy, 0.0,
                     0.0, 0.0, 1.0, 0.0]
        message.distortion_model = "plumb_bob"
        message.D = [0.0] * 5
        return message

    @staticmethod
    def fill_rect(buffer, width, height, x0, y0, x1, y1, color):
        x0, x1 = max(0, int(x0)), min(width, int(x1))
        y0, y1 = max(0, int(y0)), min(height, int(y1))
        pixel = bytes(color)
        for y in range(y0, y1):
            start = (y * width + x0) * 3
            buffer[start:start + (x1 - x0) * 3] = pixel * (x1 - x0)

    def make_camera_image(self, stamp):
        width, height = self.camera_width, self.camera_height
        data = bytearray((34, 38, 44)) * (width * height)
        dx, dy = self.tag_x - self.x, self.tag_y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        optical_x = s * dx - c * dy
        optical_z = c * dx + s * dy
        bearing = math.atan2(optical_x, optical_z)
        visible = self.tag_is_visible(optical_z, math.hypot(dx, dy), bearing)
        if visible:
            # Only render the charging panel when the same line-of-sight test
            # used by /tag_detections says that the wall does not occlude it.
            self.fill_rect(data, width, height, width*0.30, height*0.18,
                           width*0.70, height*0.92, (70, 74, 80))
            center_x = width/2.0 + self.camera_fx * optical_x / optical_z
            center_y = height*0.43
            side = clamp(self.camera_fx * self.tag_size / optical_z, 10.0, height*0.62)
            # Exact tag36h11 ID 0 bitmap sampled from the bundled reference.
            bits = (
                "0000000000", "0111111110", "0100101010", "0110001010",
                "0110011110", "0101011110", "0110100110", "0111101110",
                "0111111110", "0000000000",
            )
            cell = side / 10.0
            for row, line in enumerate(bits):
                for column, bit in enumerate(line):
                    x0 = center_x-side/2 + column*cell
                    y0 = center_y-side/2 + row*cell
                    color = (8, 8, 8) if bit == "1" else (245, 245, 245)
                    self.fill_rect(data, width, height, x0, y0,
                                   x0+cell+0.5, y0+cell+0.5, color)

        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = "camera_color_optical_frame"
        message.height, message.width = height, width
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = width * 3
        message.data = bytes(data)
        return message

    @staticmethod
    def color(r, g, b, a=1.0):
        return ColorRGBA(r, g, b, a)

    def cube_marker(self, marker_id, frame, x, y, sx, sy, color, stamp, yaw=0.0):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = frame, stamp
        marker.ns, marker.id, marker.type, marker.action = "parking", marker_id, Marker.CUBE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 0.06
        marker.pose.orientation = self.quaternion(yaw)
        marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, 0.12
        marker.color = color
        return marker

    def make_markers(self, stamp):
        markers = []
        markers.append(self.cube_marker(0, "map", self.x, self.y, self.length, self.width,
                                        self.color(0.28, 0.66, 0.20), stamp, self.yaw))
        for index, wall in enumerate(self.walls, 1):
            markers.append(self.cube_marker(index, "map", *wall, self.color(0.15, 0.35, 0.75), stamp))

        # Charging dock pad in front of the Tag (black frame + two grey contacts + bridge).
        markers.append(self.cube_marker(
            30, "map", self.pad_x, self.pad_y, self.pad_length, self.pad_width,
            self.color(0.08, 0.08, 0.10), stamp))
        contact_len = self.pad_length * 0.42
        contact_wid = self.pad_width * 0.38
        markers.append(self.cube_marker(
            31, "map", self.pad_x - self.pad_length * 0.18, self.pad_y + self.pad_width * 0.22,
            contact_len, contact_wid, self.color(0.72, 0.74, 0.76), stamp))
        markers.append(self.cube_marker(
            32, "map", self.pad_x - self.pad_length * 0.18, self.pad_y - self.pad_width * 0.22,
            contact_len, contact_wid, self.color(0.72, 0.74, 0.76), stamp))
        bridge_x = self.pad_x + 0.5 * (self.pad_length + self.bridge_length)
        markers.append(self.cube_marker(
            33, "map", bridge_x, self.pad_y, self.bridge_length, self.bridge_width,
            self.color(0.10, 0.10, 0.12), stamp))

        # Spawn region outline (2 m square).
        markers.append(self.cube_marker(
            34, "map", self.spawn_center_x, self.spawn_center_y,
            self.spawn_region_size, self.spawn_region_size,
            self.color(0.95, 0.75, 0.15, 0.18), stamp))

        goal = self.cube_marker(10, "map", self.goal[0], self.goal[1], self.length, self.width,
                                self.color(0.2, 0.85, 0.25, 0.28), stamp, self.goal[2])
        markers.append(goal)
        markers.append(self.cube_marker(12, "map", self.tag_x, self.tag_y, 0.07, 0.30,
                                        self.color(0.95, 0.55, 0.05), stamp))
        arrow = Marker()
        arrow.header.frame_id, arrow.header.stamp = "map", stamp
        arrow.ns, arrow.id, arrow.type, arrow.action = "parking", 11, Marker.ARROW, Marker.ADD
        arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = self.x, self.y, 0.14
        arrow.pose.orientation = self.quaternion(self.yaw)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.42, 0.07, 0.07
        arrow.color = self.color(1.0, 1.0, 1.0)
        markers.append(arrow)
        markers.append(self.text_marker(20, self.x, self.y, 0.20, "DOG", stamp,
                                        self.color(1.0, 1.0, 1.0)))
        markers.append(self.text_marker(21, self.back_x, 0.0, 0.22, "KENNEL", stamp,
                                        self.color(0.1, 0.2, 0.55)))
        markers.append(self.text_marker(23, self.tag_x, self.tag_y, 0.30, "TAG36H11_0 / CHARGER", stamp,
                                        self.color(0.85, 0.35, 0.02)))
        markers.append(self.text_marker(24, self.pad_x, self.pad_y - 0.45, 0.12, "CHARGE PAD", stamp,
                                        self.color(0.2, 0.2, 0.2)))
        markers.append(self.text_marker(
            25, self.spawn_center_x, self.spawn_center_y - 1.15, 0.12,
            "SPAWN 2x2 @ 2m", stamp, self.color(0.75, 0.55, 0.05)))
        status_text = "PARKED" if self.parked else ("COLLISION" if self.collision else "RUNNING")
        markers.append(self.text_marker(22, self.goal[0], -0.92, 0.24, status_text,
                                        stamp, self.color(0.05, 0.65, 0.12)))
        return MarkerArray(markers)

    def text_marker(self, marker_id, x, y, z, text, stamp, color):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = "map", stamp
        marker.ns, marker.id = "parking", marker_id
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color = color
        marker.text = text
        return marker


if __name__ == "__main__":
    rospy.init_node("parking_sim")
    try:
        ParkingSim()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logerr("parking_sim stopped: %s", error)
