#!/usr/bin/env python3

import csv
import math
import os
import struct
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def make_output_dir(base_dir: str | None) -> Path:
    if base_dir:
        output_dir = Path(base_dir)
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = Path('data') / f'romi_sensor_export_{timestamp}'

    (output_dir / 'pointclouds').mkdir(parents=True, exist_ok=True)
    (output_dir / 'lidar_scans').mkdir(parents=True, exist_ok=True)
    return output_dir


def decode_rgb(packed_value: float) -> tuple[int, int, int]:
    packed_int = struct.unpack('I', struct.pack('f', packed_value))[0]
    red = (packed_int >> 16) & 255
    green = (packed_int >> 8) & 255
    blue = packed_int & 255
    return red, green, blue


def write_ply(path: Path, points: list[tuple[float, float, float]], colors: list[tuple[int, int, int]] | None = None):
    has_color = colors is not None and len(colors) == len(points)
    with path.open('w', encoding='utf-8') as file_handle:
        file_handle.write('ply\n')
        file_handle.write('format ascii 1.0\n')
        file_handle.write(f'element vertex {len(points)}\n')
        file_handle.write('property float x\n')
        file_handle.write('property float y\n')
        file_handle.write('property float z\n')
        if has_color:
            file_handle.write('property uchar red\n')
            file_handle.write('property uchar green\n')
            file_handle.write('property uchar blue\n')
        file_handle.write('end_header\n')

        if has_color:
            for (x, y, z), (red, green, blue) in zip(points, colors):
                file_handle.write(f'{x:.6f} {y:.6f} {z:.6f} {red} {green} {blue}\n')
        else:
            for x, y, z in points:
                file_handle.write(f'{x:.6f} {y:.6f} {z:.6f}\n')


class SensorDataExporter(Node):
    def __init__(self):
        super().__init__('sensor_data_exporter')

        self.declare_parameter('cloud_topic', '/depth_camera/points')
        self.declare_parameter('scan_topic', '/lidar/scan')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('cloud_every_n', 1)
        self.declare_parameter('scan_every_n', 1)

        cloud_topic = self.get_parameter('cloud_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        output_dir = self.get_parameter('output_dir').value
        self.cloud_every_n = max(1, int(self.get_parameter('cloud_every_n').value))
        self.scan_every_n = max(1, int(self.get_parameter('scan_every_n').value))
        self.output_dir = make_output_dir(output_dir)

        self.cloud_count = 0
        self.scan_count = 0

        self.cloud_subscription = self.create_subscription(PointCloud2, cloud_topic, self.cloud_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.get_logger().info(f'Writing data into {self.output_dir}')
        self.get_logger().info('Point clouds will be written as PLY files.')
        self.get_logger().info('LiDAR scans will be written as CSV files.')

    def cloud_callback(self, msg: PointCloud2):
        self.cloud_count += 1
        if self.cloud_count % self.cloud_every_n != 0:
            return

        field_names = [field.name for field in msg.fields]
        has_color = 'rgb' in field_names or 'rgba' in field_names
        names = ['x', 'y', 'z'] + (['rgb'] if 'rgb' in field_names else ['rgba'] if 'rgba' in field_names else [])

        points: list[tuple[float, float, float]] = []
        colors: list[tuple[int, int, int]] = []

        for point in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            points.append((x, y, z))
            if has_color:
                red, green, blue = decode_rgb(float(point[3]))
                colors.append((red, green, blue))

        file_name = self.output_dir / 'pointclouds' / f'cloud_{self.cloud_count:06d}.ply'
        write_ply(file_name, points, colors if has_color else None)
        self.get_logger().info(f'Saved {file_name.name} with {len(points)} points')

    def scan_callback(self, msg: LaserScan):
        self.scan_count += 1
        if self.scan_count % self.scan_every_n != 0:
            return

        file_name = self.output_dir / 'lidar_scans' / f'scan_{self.scan_count:06d}.csv'
        with file_name.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['index', 'angle_rad', 'range_m'])
            for index, range_value in enumerate(msg.ranges):
                angle = msg.angle_min + index * msg.angle_increment
                if math.isfinite(range_value):
                    writer.writerow([index, f'{angle:.6f}', f'{range_value:.6f}'])
                else:
                    writer.writerow([index, f'{angle:.6f}', 'inf'])

        self.get_logger().info(f'Saved {file_name.name} with {len(msg.ranges)} beams')


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()