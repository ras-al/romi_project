#!/usr/bin/env python3
"""
Comprehensive sensor data recorder for the Romi robot.

Captures: point clouds (PLY), IMU, odometry, ground truth poses,
sensor transforms, and robot trajectory.

Recording is controlled via the /toggle_recording service (std_srvs/SetBool):
  ros2 service call /toggle_recording std_srvs/srv/SetBool "{data: true}"
  ros2 service call /toggle_recording std_srvs/srv/SetBool "{data: false}"
"""

import csv
import math
import os
import struct
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def decode_rgb(packed_value: float) -> tuple[int, int, int]:
    packed_int = struct.unpack('I', struct.pack('f', packed_value))[0]
    red = (packed_int >> 16) & 255
    green = (packed_int >> 8) & 255
    blue = packed_int & 255
    return red, green, blue


def write_ply(path: Path, points: list, colors: list = None):
    has_color = colors is not None and len(colors) == len(points)
    with path.open('w', encoding='utf-8') as fh:
        fh.write('ply\n')
        fh.write('format ascii 1.0\n')
        fh.write(f'element vertex {len(points)}\n')
        fh.write('property float x\nproperty float y\nproperty float z\n')
        if has_color:
            fh.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        fh.write('end_header\n')
        if has_color:
            for (x, y, z), (r, g, b) in zip(points, colors):
                fh.write(f'{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n')
        else:
            for x, y, z in points:
                fh.write(f'{x:.6f} {y:.6f} {z:.6f}\n')


class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Parameters
        self.declare_parameter('output_dir', '')
        self.declare_parameter('cloud_topic', '/depth_camera/points')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/model/romi/odometry')
        self.declare_parameter('pose_topic', '/world/world_demo/dynamic_pose/info')
        self.declare_parameter('cloud_every_n', 5)
        self.declare_parameter('auto_start', False)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cloud_every_n = max(1, int(self.get_parameter('cloud_every_n').value))
        auto_start = self.get_parameter('auto_start').value

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Recording state
        self.recording = False
        self.output_dir = None
        self.cloud_count = 0
        self.csv_files = {}
        self.csv_writers = {}

        # Service to toggle recording
        self.srv = self.create_service(SetBool, 'toggle_recording', self.toggle_recording_cb)

        # Subscriptions (always active, but only write when recording)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(PoseArray, self.pose_topic, self.pose_cb, 10)

        if auto_start:
            self._start_recording()
            self.get_logger().info(f'Data recorder auto-started → {self.output_dir}')
        else:
            self.get_logger().info('Data recorder ready. Waiting for recording trigger...')
            self.get_logger().info('  Start: ros2 service call /toggle_recording std_srvs/srv/SetBool "{data: true}"')
            self.get_logger().info('  Stop:  ros2 service call /toggle_recording std_srvs/srv/SetBool "{data: false}"')


    def toggle_recording_cb(self, request, response):
        if request.data and not self.recording:
            self._start_recording()
            response.success = True
            response.message = f'Recording started → {self.output_dir}'
        elif not request.data and self.recording:
            self._stop_recording()
            response.success = True
            response.message = 'Recording stopped and files saved.'
        elif request.data and self.recording:
            response.success = True
            response.message = f'Already recording → {self.output_dir}'
        else:
            response.success = True
            response.message = 'Not currently recording.'
        return response

    def _start_recording(self):
        output_param = self.get_parameter('output_dir').value
        if output_param:
            self.output_dir = Path(output_param)
        else:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output_dir = Path('data') / f'romi_capture_{timestamp}'

        (self.output_dir / 'pointclouds').mkdir(parents=True, exist_ok=True)

        # Open CSV files with headers
        self._open_csv('trajectory', ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self._open_csv('imu', ['timestamp', 'ax', 'ay', 'az', 'wx', 'wy', 'wz', 'qx', 'qy', 'qz', 'qw'])
        self._open_csv('odometry', [
            'timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
            'vx', 'vy', 'vz', 'wx', 'wy', 'wz'
        ])
        self._open_csv('ground_truth', ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self._open_csv('transforms', [
            'timestamp', 'parent_frame', 'child_frame',
            'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'
        ])

        self.cloud_count = 0
        self.recording = True
        self.get_logger().info(f'▶ Recording started → {self.output_dir}')

    def _stop_recording(self):
        self.recording = False
        for name, fh in self.csv_files.items():
            fh.close()
        self.csv_files.clear()
        self.csv_writers.clear()
        self.get_logger().info(f'⏹ Recording stopped. {self.cloud_count} point clouds saved.')

    def _open_csv(self, name: str, header: list):
        path = self.output_dir / f'{name}.csv'
        fh = open(path, 'w', newline='', encoding='utf-8')
        writer = csv.writer(fh)
        writer.writerow(header)
        self.csv_files[name] = fh
        self.csv_writers[name] = writer

    def _stamp_to_float(self, stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    # ── Point Cloud callback ──────────────────────────────────────
    def cloud_cb(self, msg: PointCloud2):
        if not self.recording:
            return
        self.cloud_count += 1
        if self.cloud_count % self.cloud_every_n != 0:
            return

        # Extract points
        field_names = [f.name for f in msg.fields]
        has_color = 'rgb' in field_names or 'rgba' in field_names
        names = ['x', 'y', 'z'] + (['rgb'] if 'rgb' in field_names else ['rgba'] if 'rgba' in field_names else [])

        points = []
        colors = []
        for pt in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
            points.append((float(pt[0]), float(pt[1]), float(pt[2])))
            if has_color:
                colors.append(decode_rgb(float(pt[3])))

        ply_path = self.output_dir / 'pointclouds' / f'cloud_{self.cloud_count:06d}.ply'
        write_ply(ply_path, points, colors if has_color else None)

        # Look up sensor transform
        ts = self._stamp_to_float(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp,
                                                  rclpy.duration.Duration(seconds=0.1))
            t = tf.transform.translation
            r = tf.transform.rotation
            self.csv_writers['transforms'].writerow([
                f'{ts:.6f}', tf.header.frame_id, tf.child_frame_id,
                f'{t.x:.6f}', f'{t.y:.6f}', f'{t.z:.6f}',
                f'{r.x:.6f}', f'{r.y:.6f}', f'{r.z:.6f}', f'{r.w:.6f}'
            ])
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)

        self.get_logger().info(f'Saved {ply_path.name} ({len(points)} pts)', throttle_duration_sec=2.0)

    # ── IMU callback ──────────────────────────────────────────────
    def imu_cb(self, msg: Imu):
        if not self.recording:
            return
        ts = self._stamp_to_float(msg.header.stamp)
        a = msg.linear_acceleration
        w = msg.angular_velocity
        q = msg.orientation
        self.csv_writers['imu'].writerow([
            f'{ts:.6f}',
            f'{a.x:.6f}', f'{a.y:.6f}', f'{a.z:.6f}',
            f'{w.x:.6f}', f'{w.y:.6f}', f'{w.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}'
        ])

    # ── Odometry callback ─────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        if not self.recording:
            return
        ts = self._stamp_to_float(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.csv_writers['odometry'].writerow([
            f'{ts:.6f}',
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}',
            f'{v.x:.6f}', f'{v.y:.6f}', f'{v.z:.6f}',
            f'{w.x:.6f}', f'{w.y:.6f}', f'{w.z:.6f}'
        ])
        # Also log trajectory (position + orientation only)
        self.csv_writers['trajectory'].writerow([
            f'{ts:.6f}',
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}'
        ])

    # ── Ground Truth Pose callback ────────────────────────────────
    def pose_cb(self, msg: PoseArray):
        if not self.recording:
            return
        ts = self._stamp_to_float(msg.header.stamp)
        # PoseArray from PosePublisher contains poses for all dynamic models
        # The romi model pose is typically one of the entries
        for pose in msg.poses:
            p = pose.position
            q = pose.orientation
            self.csv_writers['ground_truth'].writerow([
                f'{ts:.6f}',
                f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
                f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}'
            ])


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.recording:
            node._stop_recording()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
