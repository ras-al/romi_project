#!/usr/bin/env python3

import math
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class WarehouseExplorer(Node):
    def __init__(self):
        super().__init__('warehouse_explorer')

        self.declare_parameter('cmd_vel_topic', '/model/romi/cmd_vel')
        self.declare_parameter('scan_topic', '/lidar/scan')
        self.declare_parameter('cruise_speed', 0.12)
        self.declare_parameter('reverse_speed', 0.08)
        self.declare_parameter('turn_speed', 1.2)
        self.declare_parameter('turn_gain', 0.6)
        self.declare_parameter('front_stop_distance', 0.85)
        self.declare_parameter('front_reverse_distance', 0.35)
        self.declare_parameter('wall_distance', 1.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_reverse_distance = float(self.get_parameter('front_reverse_distance').value)
        self.wall_distance = float(self.get_parameter('wall_distance').value)

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.subscription = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.timer = self.create_timer(0.10, self.control_loop)

        self.latest_scan = None
        self.last_reaction = ''
        self.active = False
        self.running = True

        self.get_logger().info('Warehouse explorer ready.')
        self.get_logger().info('Type: start, stop, status, quit')

        self.command_thread = threading.Thread(target=self.command_loop, daemon=True)
        self.command_thread.start()

    def command_loop(self):
        while self.running and rclpy.ok():
            try:
                command = input('romi> ').strip().lower()
            except EOFError:
                command = 'quit'

            if not command:
                continue

            if command in ('start', 'go', 'run'):
                self.active = True
                self.get_logger().info('Autonomous movement enabled.')
            elif command in ('stop', 'halt'):
                self.active = False
                self.publish_stop()
                self.get_logger().info('Robot stopped.')
            elif command == 'status':
                state = 'active' if self.active else 'stopped'
                self.get_logger().info(f'Status: {state}')
            elif command in ('quit', 'exit'):
                self.running = False
                self.active = False
                self.publish_stop()
                rclpy.shutdown()
                return
            else:
                self.get_logger().info('Commands: start, stop, status, quit')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def publish_stop(self):
        twist = Twist()
        self.publisher.publish(twist)

    def sector_min(self, scan: LaserScan, start_angle: float, end_angle: float) -> float:
        if scan.angle_increment <= 0.0 or not scan.ranges:
            return float('inf')

        lower = min(start_angle, end_angle)
        upper = max(start_angle, end_angle)
        start_index = int((lower - scan.angle_min) / scan.angle_increment)
        end_index = int((upper - scan.angle_min) / scan.angle_increment)
        start_index = max(0, start_index)
        end_index = min(len(scan.ranges) - 1, end_index)

        values = []
        for index in range(start_index, end_index + 1):
            value = scan.ranges[index]
            if math.isfinite(value):
                values.append(value)
        return min(values) if values else float('inf')

    def log_reaction(self, message: str):
        if message != self.last_reaction:
            self.last_reaction = message
            self.get_logger().info(message)

    def control_loop(self):
        if not self.active or self.latest_scan is None:
            return

        scan = self.latest_scan
        front = self.sector_min(scan, -0.60, 0.60)
        front_left = self.sector_min(scan, 0.20, 1.00)
        front_right = self.sector_min(scan, -1.00, -0.20)
        left = self.sector_min(scan, 0.80, 1.80)
        right = self.sector_min(scan, -1.80, -0.80)

        twist = Twist()

        if front < self.front_stop_distance or min(front_left, front_right) < self.front_stop_distance:
            if front < self.front_reverse_distance or min(front_left, front_right) < self.front_reverse_distance:
                twist.linear.x = -self.reverse_speed
            else:
                twist.linear.x = 0.0

            if left >= right:
                twist.angular.z = self.turn_speed
                self.log_reaction(
                    f'Obstacle ahead: front={front:.2f} front_left={front_left:.2f} front_right={front_right:.2f} '
                    f'left={left:.2f} right={right:.2f} -> turning left'
                )
            else:
                twist.angular.z = -self.turn_speed
                self.log_reaction(
                    f'Obstacle ahead: front={front:.2f} front_left={front_left:.2f} front_right={front_right:.2f} '
                    f'left={left:.2f} right={right:.2f} -> turning right'
                )
        else:
            steer = clamp((right - left) * self.turn_gain, -self.turn_speed, self.turn_speed)
            twist.linear.x = self.cruise_speed
            twist.angular.z = steer

            if min(left, right) < self.wall_distance:
                twist.linear.x = min(self.cruise_speed, 0.12)
                self.log_reaction(
                    f'Wall following: front={front:.2f} front_left={front_left:.2f} front_right={front_right:.2f} '
                    f'left={left:.2f} right={right:.2f} steer={steer:.2f}'
                )
            else:
                self.last_reaction = ''

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()