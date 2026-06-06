#!/usr/bin/env python3
"""
Improved autonomous explorer for the Romi robot.
 
Fixes over v1:
  - Stuck detection: if the robot hasn't moved in N seconds, it forces a recovery spin
  - Better front-sector split: narrow forward cone + left/right side zones
  - Wall-following hysteresis to stop oscillating in corridors
  - Configurable exploration timeout (stops recording cleanly)
  - Publishes /exploration_status (std_msgs/String) for monitoring
"""
 
import math
import random
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
 
 
class State:
    WAITING    = 'WAITING'
    EXPLORING  = 'EXPLORING'
    TURNING    = 'TURNING'
    REVERSING  = 'REVERSING'
    RECOVERING = 'RECOVERING'   # forced spin when stuck
 
 
class ReactiveExplorer(Node):
    def __init__(self):
        super().__init__('reactive_explorer')
 
        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic',       '/model/romi/cmd_vel')
        self.declare_parameter('scan_topic',          '/lidar/scan')
        self.declare_parameter('odom_topic',          '/model/romi/odometry')
        self.declare_parameter('linear_speed',         0.18)
        self.declare_parameter('angular_speed',        0.7)
        self.declare_parameter('obstacle_threshold',   0.50)  # metres
        self.declare_parameter('side_threshold',       0.25)  # wall-follow margin
        self.declare_parameter('turn_duration',        2.0)   # seconds
        self.declare_parameter('stuck_timeout',        4.0)   # seconds without movement
        self.declare_parameter('stuck_move_threshold', 0.04)  # metres — below = stuck
        self.declare_parameter('exploration_timeout',  0.0)   # 0 = unlimited
        # self.declare_parameter('use_sim_time',         True) # Automatically declared by launch file
 
        self.linear_speed         = self.get_parameter('linear_speed').value
        self.angular_speed        = self.get_parameter('angular_speed').value
        self.obstacle_threshold   = self.get_parameter('obstacle_threshold').value
        self.side_threshold       = self.get_parameter('side_threshold').value
        self.turn_duration        = self.get_parameter('turn_duration').value
        self.stuck_timeout        = self.get_parameter('stuck_timeout').value
        self.stuck_move_threshold = self.get_parameter('stuck_move_threshold').value
        self.exploration_timeout  = self.get_parameter('exploration_timeout').value
 
        # ── Publishers ────────────────────────────────────────────
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub    = self.create_publisher(Twist,  cmd_topic, 10)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
 
        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
 
        # ── State machine ─────────────────────────────────────────
        self.state            = State.WAITING
        self.latest_scan      = None
        self.recording_active = False
 
        self.state_timer      = 0.0
        self.turn_direction   = 1.0   # +1 left, -1 right
 
        # Stuck detection
        self.last_x           = None
        self.last_y           = None
        self.last_move_time   = time.monotonic()
 
        # Exploration timeout
        self.start_time       = None
 
        # Wall-follow hysteresis: don't switch sides every tick
        self._wall_follow_dir = 0.0   # 0 = none, +1 = drifting right, -1 = left
 
        # ── 10 Hz control loop ────────────────────────────────────
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.control_loop)
 
        self.get_logger().info('ReactiveExplorer v2 ready — waiting for LiDAR...')
 
    # ─────────────────────────────────────────────────────────────
    # Subscribers
    # ─────────────────────────────────────────────────────────────
 
    def scan_cb(self, msg: LaserScan):
        if self.state == State.WAITING:
            self.get_logger().info('LiDAR received — starting exploration')
            self.state      = State.EXPLORING
            self.start_time = time.monotonic()
            self._trigger_recording(start=True)
        self.latest_scan = msg
 
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_x is None:
            self.last_x, self.last_y = x, y
            return
        dist = math.hypot(x - self.last_x, y - self.last_y)
        if dist > self.stuck_move_threshold:
            self.last_move_time = time.monotonic()
            self.last_x, self.last_y = x, y
 
    # ─────────────────────────────────────────────────────────────
    # LiDAR helpers
    # ─────────────────────────────────────────────────────────────
 
    def sector_min(self, scan: LaserScan, deg_start: float, deg_end: float) -> float:
        """Minimum finite range in a degree arc (robot-centric, 0° = forward)."""
        if scan is None:
            return float('inf')
        a_min = math.radians(deg_start)
        a_max = math.radians(deg_end)
        i0 = max(0, int((a_min - scan.angle_min) / scan.angle_increment))
        i1 = min(len(scan.ranges)-1, int((a_max - scan.angle_min) / scan.angle_increment))
        if i0 > i1:
            i0, i1 = i1, i0
        vals = [r for r in scan.ranges[i0:i1+1]
                if math.isfinite(r) and scan.range_min < r < scan.range_max]
        return min(vals) if vals else float('inf')
 
    # ─────────────────────────────────────────────────────────────
    # Recording service
    # ─────────────────────────────────────────────────────────────
 
    def _trigger_recording(self, start: bool):
        if start and self.recording_active:
            return
        client = self.create_client(SetBool, 'toggle_recording')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('toggle_recording service not found')
            return
        req = SetBool.Request()
        req.data = start
        fut = client.call_async(req)
        fut.add_done_callback(
            lambda f: self.get_logger().info(
                f'Recorder: {f.result().message}' if f.result() else 'Recorder call failed'))
        self.recording_active = start
 
    # ─────────────────────────────────────────────────────────────
    # Control loop
    # ─────────────────────────────────────────────────────────────
 
    def control_loop(self):
        if self.state == State.WAITING or self.latest_scan is None:
            return
 
        # ── Exploration timeout ───────────────────────────────────
        if self.exploration_timeout > 0 and self.start_time is not None:
            elapsed = time.monotonic() - self.start_time
            if elapsed > self.exploration_timeout:
                self.get_logger().info('Exploration timeout reached — stopping.')
                self._trigger_recording(start=False)
                self.cmd_pub.publish(Twist())
                self.state = State.WAITING
                return
 
        scan = self.latest_scan
        twist = Twist()
 
        # Sensor zones
        front       = self.sector_min(scan, -20,  20)
        front_left  = self.sector_min(scan,  20,  60)
        front_right = self.sector_min(scan, -60, -20)
        left        = self.sector_min(scan,  60,  120)
        right       = self.sector_min(scan, -120, -60)
        rear        = self.sector_min(scan,  150, 180)
 
        thr = self.obstacle_threshold
 
        # ── Stuck detection ───────────────────────────────────────
        if self.state == State.EXPLORING:
            if (time.monotonic() - self.last_move_time) > self.stuck_timeout:
                self.get_logger().warn('Stuck detected — initiating recovery spin')
                self.state       = State.RECOVERING
                self.state_timer = 3.0
                # Spin away from the side with the closer wall
                self.turn_direction = 1.0 if right < left else -1.0
                self.last_move_time = time.monotonic()  # reset
 
        # ── State machine ─────────────────────────────────────────
 
        if self.state == State.EXPLORING:
            if front < thr:
                if front < 0.20:
                    # Emergency reverse
                    self.state       = State.REVERSING
                    self.state_timer = 1.2
                    self.get_logger().info(f'Too close ({front:.2f} m) — reversing')
                else:
                    # Obstacle: decide which way to turn
                    self.state       = State.TURNING
                    jitter           = 0.6 + 0.8 * random.random()
                    self.state_timer = self.turn_duration * jitter
 
                    # Prefer the side with more clearance
                    open_left  = min(front_left,  left)
                    open_right = min(front_right, right)
                    self.turn_direction = 1.0 if open_left >= open_right else -1.0
 
                    side = 'LEFT' if self.turn_direction > 0 else 'RIGHT'
                    self.get_logger().info(
                        f'Obstacle {front:.2f} m — turning {side} for {self.state_timer:.1f}s')
            else:
                # ── Forward + gentle wall-follow ──────────────────
                twist.linear.x = self.linear_speed
 
                # Hysteresis: only activate wall-follow when clearly too close
                if left < self.side_threshold:
                    self._wall_follow_dir = -0.25   # drift right
                elif right < self.side_threshold:
                    self._wall_follow_dir = +0.25   # drift left
                else:
                    # Fade back to centre gradually
                    self._wall_follow_dir *= 0.85
                    if abs(self._wall_follow_dir) < 0.02:
                        self._wall_follow_dir = 0.0
 
                # Small random perturbation to avoid infinite straight lines
                noise = random.uniform(-0.08, 0.08)
                twist.angular.z = self._wall_follow_dir + noise
 
        elif self.state == State.TURNING:
            self.state_timer -= self.timer_period
            twist.angular.z   = self.angular_speed * self.turn_direction
 
            # Early exit if path is clear enough
            clear_ahead = front > thr * 1.6
            if self.state_timer <= 0 or clear_ahead:
                self.state = State.EXPLORING
                self.get_logger().info('Turn done — resuming exploration')
 
        elif self.state == State.REVERSING:
            self.state_timer -= self.timer_period
            twist.linear.x    = -self.linear_speed * 0.7
            # Curve a bit while reversing
            twist.angular.z   = 0.3 * self.turn_direction
            if self.state_timer <= 0:
                self.state       = State.TURNING
                self.state_timer = self.turn_duration
                self.get_logger().info('Reverse done — turning')
 
        elif self.state == State.RECOVERING:
            # Spin in place until path is clear
            self.state_timer -= self.timer_period
            twist.angular.z   = self.angular_speed * self.turn_direction
            if self.state_timer <= 0 or front > thr * 1.2:
                self.state = State.EXPLORING
                self.get_logger().info('Recovery done — resuming exploration')
 
        self.cmd_pub.publish(twist)
 
        # ── Status topic ──────────────────────────────────────────
        status_msg = String()
        status_msg.data = (
            f'state={self.state} front={front:.2f} '
            f'left={left:.2f} right={right:.2f}'
        )
        self.status_pub.publish(status_msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ReactiveExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())   # stop robot
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
 
 
if __name__ == '__main__':
    main()
