#!/usr/bin/env python3
"""
Reactive autonomous exploration for the Romi robot.

Uses a simple wall-following / random-walk strategy to explore the environment
safely using only LiDAR data. This avoids coordinate frame issues between
odometry and SLAM, ensuring stable continuous movement.

Automatically triggers the data recorder when exploration starts.
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class State:
    WAITING = 'WAITING'
    EXPLORING = 'EXPLORING'
    TURNING = 'TURNING'
    REVERSING = 'REVERSING'


class ReactiveExplorer(Node):
    def __init__(self):
        super().__init__('reactive_explorer')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic', '/model/romi/cmd_vel')
        self.declare_parameter('scan_topic', '/lidar/scan')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('obstacle_threshold', 0.45)
        self.declare_parameter('turn_duration', 1.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.turn_duration = self.get_parameter('turn_duration').value

        # ── Publishers ────────────────────────────────────────────
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ── Subscribers ───────────────────────────────────────────
        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)

        # ── State ─────────────────────────────────────────────────
        self.state = State.WAITING
        self.latest_scan = None
        self.recording_started = False
        
        # State timers
        self.state_timer = 0.0
        self.turn_direction = 1.0  # 1 for left, -1 for right

        # ── Control loop at 10 Hz ─────────────────────────────────
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Reactive Explorer initialized. Waiting for LiDAR...')

    # ── Callbacks ─────────────────────────────────────────────────

    def scan_cb(self, msg: LaserScan):
        if self.state == State.WAITING:
            self.get_logger().info('LiDAR received! Starting exploration...')
            self.state = State.EXPLORING
            self._trigger_recording()
            
        self.latest_scan = msg

    # ── Recording trigger ─────────────────────────────────────────

    def _trigger_recording(self):
        if self.recording_started:
            return
        self.recording_started = True

        client = self.create_client(SetBool, 'toggle_recording')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Data recorder service not available, skipping auto-start.')
            return

        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Data recording: {f.result().message}' if f.result() else 'Recording call failed'))

    # ── LiDAR processing ──────────────────────────────────────────

    def get_sector_min(self, scan: LaserScan, start_deg: float, end_deg: float) -> float:
        """Get minimum range in a sector defined by degrees."""
        if scan is None:
            return float('inf')
            
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        start_idx = max(0, int((start_rad - scan.angle_min) / scan.angle_increment))
        end_idx = min(len(scan.ranges) - 1, int((end_rad - scan.angle_min) / scan.angle_increment))
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        values = [r for r in scan.ranges[start_idx:end_idx + 1] if math.isfinite(r) and r > 0.05]
        return min(values) if values else float('inf')

    # ── Control Loop ──────────────────────────────────────────────

    def control_loop(self):
        if self.state == State.WAITING or self.latest_scan is None:
            return

        twist = Twist()
        
        # Check obstacle sectors
        front_dist = self.get_sector_min(self.latest_scan, -25, 25)
        left_dist = self.get_sector_min(self.latest_scan, 25, 90)
        right_dist = self.get_sector_min(self.latest_scan, -90, -25)

        if self.state == State.EXPLORING:
            if front_dist < self.obstacle_threshold:
                # Obstacle ahead! Decide which way to turn
                if front_dist < 0.2:
                    # Too close! Reverse
                    self.state = State.REVERSING
                    self.state_timer = 1.0  # Reverse for 1 second
                    self.get_logger().info(f'Too close ({front_dist:.2f}m)! Reversing...')
                else:
                    self.state = State.TURNING
                    # Turn towards the more open side, add some randomness
                    base_turn_time = self.turn_duration * (0.8 + 0.4 * random.random())
                    self.state_timer = base_turn_time
                    
                    if left_dist > right_dist:
                        self.turn_direction = 1.0  # Turn left
                        self.get_logger().info(f'Obstacle ahead ({front_dist:.2f}m). Turning LEFT for {base_turn_time:.1f}s')
                    else:
                        self.turn_direction = -1.0 # Turn right
                        self.get_logger().info(f'Obstacle ahead ({front_dist:.2f}m). Turning RIGHT for {base_turn_time:.1f}s')
            else:
                # Clear path, move forward
                twist.linear.x = self.linear_speed
                
                # Slight wall-following tendency / drift correction
                if left_dist < 0.3:
                    twist.angular.z = -0.2  # Drift right
                elif right_dist < 0.3:
                    twist.angular.z = 0.2   # Drift left
                else:
                    # Occasional slight random drift to encourage area coverage
                    twist.angular.z = random.uniform(-0.1, 0.1)

        elif self.state == State.TURNING:
            self.state_timer -= self.timer_period
            
            # Keep turning
            twist.angular.z = self.angular_speed * self.turn_direction
            
            # If path is VERY clear and we've turned a bit, we can exit early
            if self.state_timer <= 0 or front_dist > self.obstacle_threshold * 1.5:
                self.state = State.EXPLORING
                self.get_logger().info('Turn complete. Resuming exploration.')

        elif self.state == State.REVERSING:
            self.state_timer -= self.timer_period
            
            twist.linear.x = -self.linear_speed * 0.8
            twist.angular.z = self.angular_speed * self.turn_direction # curve while reversing
            
            if self.state_timer <= 0:
                self.state = State.TURNING
                self.state_timer = self.turn_duration
                self.get_logger().info('Reverse complete. Now turning.')

        self.cmd_pub.publish(twist)

    def _stop_recording(self):
        client = self.create_client(SetBool, 'toggle_recording')
        if not client.wait_for_service(timeout_sec=2.0):
            return
        request = SetBool.Request()
        request.data = False
        client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        try:
            stop = Twist()
            node.cmd_pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
