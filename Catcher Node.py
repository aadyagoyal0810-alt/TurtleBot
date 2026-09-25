#!/usr/bin/env python3
"""
catcher_node.py
----------------
Catcher (pursuit) algorithm for the TurtleBot Pursuit & Evasion Challenge
(Tech Zephyr 4.0, IIT Bhubaneswar, Round 1 - Catcher Qualification).

STRATEGY
--------
1. Track the Runner's position (moving sphere/dot or organizer-controlled TurtleBot).
2. Continuously compute a heading toward the Runner and drive toward it
   (pure-pursuit style: turn-then-drive, blended so the robot doesn't stop
   to rotate in place).
3. Use LiDAR (/scan) for reactive obstacle avoidance so the Catcher doesn't
   get stuck on arena obstacles while chasing.
4. Declare "capture" when within CAPTURE_RADIUS of the Runner for a short
   confirmation window (avoids false positives from a single close frame).

⚠️ IMPORTANT — THINGS YOU MUST CONFIRM/ADJUST BEFORE SUBMITTING ⚠️
-------------------------------------------------------------------
This was written using the most common ROS2 defaults, since the exact
interface spec (topic names, message types, arena size, target format)
wasn't available from the page you shared. Check the competition's
FAQs & Discussions tab / rulebook / starter repo for the *actual* values
and update the CONFIG block below accordingly:

  - SELF_ODOM_TOPIC   : where your own robot's odometry is published.
  - TARGET_TOPIC       : where the Runner's position is published. If the
                          organizers only give you /scan or a camera feed
                          instead of a direct pose topic, you'll need to
                          add detection logic (this file assumes you
                          receive geometry_msgs/PoseStamped or
                          nav_msgs/Odometry for the target directly).
  - CMD_VEL_TOPIC      : output velocity command topic.
  - MAX_LINEAR_SPEED / MAX_ANGULAR_SPEED : per the platform spec
    (TurtleBot3 Burger ~0.22 m/s / 2.84 rad/s; TurtleBot4 Lite ~0.31 m/s
    / 1.9 rad/s -- confirm against the rulebook).
  - CAPTURE_RADIUS     : the official "capture" distance, if specified.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ----------------------------- CONFIG -------------------------------------
SELF_ODOM_TOPIC = "/odom"                 # own robot pose
TARGET_TOPIC = "/target_pose"             # runner's pose  <-- CONFIRM THIS
CMD_VEL_TOPIC = "/cmd_vel"
SCAN_TOPIC = "/scan"

MAX_LINEAR_SPEED = 0.30      # m/s   <-- CONFIRM against platform spec
MAX_ANGULAR_SPEED = 1.80     # rad/s <-- CONFIRM against platform spec

CAPTURE_RADIUS = 0.30        # m, distance counted as "caught"
CAPTURE_HOLD_TIME = 1.0      # s, must stay within radius this long to confirm

OBSTACLE_STOP_DISTANCE = 0.35   # m, hard stop / avoid distance
OBSTACLE_SLOW_DISTANCE = 0.70   # m, start steering away
CONTROL_HZ = 10.0
# ----------------------------------------------------------------------------


def yaw_from_quaternion(q):
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CatcherNode(Node):
    def __init__(self):
        super().__init__("catcher_node")

        # State
        self.self_x = None
        self.self_y = None
        self.self_yaw = None

        self.target_x = None
        self.target_y = None
        self.have_target = False

        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        self.captured = False
        self.time_within_radius = 0.0

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, SELF_ODOM_TOPIC, self.odom_cb, 10)
        self.create_subscription(PoseStamped, TARGET_TOPIC, self.target_cb, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        # Control loop
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info("Catcher node started. Waiting for odom + target...")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def odom_cb(self, msg: Odometry):
        self.self_x = msg.pose.pose.position.x
        self.self_y = msg.pose.pose.position.y
        self.self_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def target_cb(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.have_target = True

    def scan_cb(self, msg: LaserScan):
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    # ------------------------------------------------------------------ #
    # Obstacle avoidance helper
    # ------------------------------------------------------------------ #
    def obstacle_avoidance_bias(self):
        """
        Returns an angular-velocity bias (rad/s) to steer away from the
        nearest close obstacle, and a speed scale factor (0..1) to slow
        down near obstacles. Uses a simple reactive scheme over the LiDAR.
        """
        if not self.scan_ranges:
            return 0.0, 1.0

        min_dist = float("inf")
        min_idx = None
        for i, r in enumerate(self.scan_ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r < min_dist:
                min_dist = r
                min_idx = i

        if min_idx is None or min_dist > OBSTACLE_SLOW_DISTANCE:
            return 0.0, 1.0

        obstacle_angle = self.scan_angle_min + min_idx * self.scan_angle_increment
        obstacle_angle = normalize_angle(obstacle_angle)

        # Steer away from the obstacle: turn opposite to its bearing.
        # Stronger bias the closer the obstacle is.
        closeness = max(0.0, (OBSTACLE_SLOW_DISTANCE - min_dist) / OBSTACLE_SLOW_DISTANCE)
        angular_bias = -math.copysign(1.0, obstacle_angle) * closeness * MAX_ANGULAR_SPEED

        speed_scale = 1.0
        if min_dist < OBSTACLE_STOP_DISTANCE:
            speed_scale = 0.0  # hard stop, just rotate away
        elif min_dist < OBSTACLE_SLOW_DISTANCE:
            speed_scale = (min_dist - OBSTACLE_STOP_DISTANCE) / (
                OBSTACLE_SLOW_DISTANCE - OBSTACLE_STOP_DISTANCE
            )

        return angular_bias, speed_scale

    # ------------------------------------------------------------------ #
    # Main control loop
    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self.captured:
            self.cmd_pub.publish(Twist())  # stay stopped
            return

        if self.self_x is None or not self.have_target:
            return  # waiting on data

        dx = self.target_x - self.self_x
        dy = self.target_y - self.self_y
        distance = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - self.self_yaw)

        # Capture check
        if distance <= CAPTURE_RADIUS:
            self.time_within_radius += 1.0 / CONTROL_HZ
            if self.time_within_radius >= CAPTURE_HOLD_TIME:
                self.captured = True
                self.get_logger().info("TARGET CAPTURED.")
                self.cmd_pub.publish(Twist())
                return
        else:
            self.time_within_radius = 0.0

        # Pure-pursuit style control: turn toward target, drive forward,
        # scaled down proportional to heading error so it doesn't overshoot.
        angular_bias, obstacle_speed_scale = self.obstacle_avoidance_bias()

        angular_z = max(
            -MAX_ANGULAR_SPEED,
            min(MAX_ANGULAR_SPEED, 2.0 * heading_error + angular_bias),
        )

        # Slow down when turning sharply or near an obstacle; speed up when
        # roughly facing the target and path is clear.
        heading_speed_scale = max(0.0, 1.0 - abs(heading_error) / (math.pi / 2))
        linear_x = MAX_LINEAR_SPEED * heading_speed_scale * obstacle_speed_scale

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
