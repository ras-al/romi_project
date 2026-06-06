#!/usr/bin/env python3
import csv
import json
import math
import os
import struct
from datetime import datetime
from pathlib import Path
 
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from std_srvs.srv import SetBool
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException,
                     ExtrapolationException)
 
 
# ── PLY helpers ──────────────────────────────────────────────────
 
def write_ply_binary(path: Path, points: list, colors: list = None):
    """Write a binary little-endian PLY — much faster than ASCII for large clouds."""
    has_color = colors is not None and len(colors) == len(points)
    with path.open('wb') as fh:
        header = ['ply', 'format binary_little_endian 1.0',
                  f'element vertex {len(points)}',
                  'property float x', 'property float y', 'property float z']
        if has_color:
            header += ['property uchar red',
                       'property uchar green',
                       'property uchar blue']
        header.append('end_header')
        fh.write(('\n'.join(header) + '\n').encode())
        if has_color:
            fmt = '<fff3B'
            for (x, y, z), (r, g, b) in zip(points, colors):
                fh.write(struct.pack(fmt, x, y, z, r, g, b))
        else:
            fmt = '<fff'
            for (x, y, z) in points:
                fh.write(struct.pack(fmt, x, y, z))
 
 
def decode_rgb(packed: float):
    i = struct.unpack('I', struct.pack('f', packed))[0]
    return (i >> 16) & 0xFF, (i >> 8) & 0xFF, i & 0xFF
 
 
# ── COLMAP pose helpers ───────────────────────────────────────────
 
def quat_to_colmap(qx, qy, qz, qw):
    """COLMAP stores quaternion as (qw, qx, qy, qz)."""
    return qw, qx, qy, qz
 
 
def pose_to_colmap_extrinsic(tx, ty, tz, qx, qy, qz, qw):
    """
    COLMAP images.txt format requires the camera-to-world transform
    expressed as the ROTATION part only (t is translation of camera in world).
    For a pure odometry pose this is trivial — R = rotation(q), t = position.
    """
    return qw, qx, qy, qz, tx, ty, tz
 
 
# ─────────────────────────────────────────────────────────────────
 
class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
 
        self.declare_parameter('output_dir',        '')
        self.declare_parameter('cloud_topic',       '/depth_camera/points')
        self.declare_parameter('image_topic',       '/depth_camera/image')
        self.declare_parameter('cam_info_topic',    '/depth_camera/camera_info')
        self.declare_parameter('imu_topic',         '/imu')
        self.declare_parameter('odom_topic',        '/model/romi/odometry')
        self.declare_parameter('pose_topic',        '/world/world_demo/dynamic_pose/info')
        self.declare_parameter('cloud_every_n',     3)   # save every 3rd cloud
        self.declare_parameter('auto_start',        True)  # start immediately
 
        self.cloud_topic    = self.get_parameter('cloud_topic').value
        self.image_topic    = self.get_parameter('image_topic').value
        self.cam_info_topic = self.get_parameter('cam_info_topic').value
        self.imu_topic      = self.get_parameter('imu_topic').value
        self.odom_topic     = self.get_parameter('odom_topic').value
        self.pose_topic     = self.get_parameter('pose_topic').value
        self.cloud_every_n  = max(1, int(self.get_parameter('cloud_every_n').value))
        auto_start          = self.get_parameter('auto_start').value
 
        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        # State
        self.recording     = False
        self.output_dir    = None
        self.cloud_count   = 0
        self.image_count   = 0
        self.frame_idx     = 0          # sequential COLMAP image index
        self.csv_files     = {}
        self.csv_writers   = {}
        self._cam_info_saved = False
        self._latest_image = None       # cache last RGB image
        self._romi_idx     = -1         # index in PoseArray for romi
 
        # Service
        self.srv = self.create_service(
            SetBool, 'toggle_recording', self.toggle_cb)
 
        # Subscriptions
        self.create_subscription(PointCloud2, self.cloud_topic,    self.cloud_cb,    10)
        self.create_subscription(Image,       self.image_topic,    self.image_cb,    10)
        self.create_subscription(CameraInfo,  self.cam_info_topic, self.cam_info_cb, 10)
        self.create_subscription(Imu,         self.imu_topic,      self.imu_cb,      10)
        self.create_subscription(Odometry,    self.odom_topic,     self.odom_cb,     10)
        self.create_subscription(PoseArray,   self.pose_topic,     self.gt_cb,       10)
 
        if auto_start:
            self._start()
        else:
            self.get_logger().info(
                'DataRecorder ready — call /toggle_recording (true) to start')
 
    # ── toggle service ───────────────────────────────────────────
 
    def toggle_cb(self, req, res):
        if req.data and not self.recording:
            self._start()
            res.success = True
            res.message = f'Recording → {self.output_dir}'
        elif not req.data and self.recording:
            self._stop()
            res.success = True
            res.message = 'Recording stopped.'
        else:
            res.success = True
            res.message = 'Already in requested state.'
        return res
 
    # ── start / stop ─────────────────────────────────────────────
 
    def _start(self):
        p = self.get_parameter('output_dir').value
        if p:
            self.output_dir = Path(p)
        else:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.output_dir = Path('data') / f'romi_capture_{ts}'
 
        (self.output_dir / 'pointclouds').mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'images').mkdir(parents=True, exist_ok=True)
 
        self._open_csv('trajectory',  ['timestamp', 'x', 'y', 'z',
                                        'qx', 'qy', 'qz', 'qw'])
        self._open_csv('imu',         ['timestamp', 'ax', 'ay', 'az',
                                        'wx', 'wy', 'wz',
                                        'qx', 'qy', 'qz', 'qw'])
        self._open_csv('odometry',    ['timestamp', 'x', 'y', 'z',
                                        'qx', 'qy', 'qz', 'qw',
                                        'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
        self._open_csv('ground_truth', ['timestamp', 'x', 'y', 'z',
                                         'qx', 'qy', 'qz', 'qw'])
        self._open_csv('transforms',  ['timestamp', 'parent_frame',
                                        'child_frame', 'tx', 'ty', 'tz',
                                        'qx', 'qy', 'qz', 'qw'])
 
        # COLMAP images.txt  (header comment)
        self._colmap_f = open(
            self.output_dir / 'images.txt', 'w', encoding='utf-8')
        self._colmap_f.write(
            '# COLMAP images.txt  IMAGE_ID QW QX QY QZ TX TY TZ '
            'CAMERA_ID NAME\n# (one blank line per image, no 2D points)\n')
 
        self.cloud_count       = 0
        self.image_count       = 0
        self.frame_idx         = 0
        self.recording         = True
        self._cam_info_saved   = False
        self.get_logger().info(f'▶ Recording → {self.output_dir}')
 
    def _stop(self):
        self.recording = False
        for fh in self.csv_files.values():
            fh.close()
        self.csv_files.clear()
        self.csv_writers.clear()
        if hasattr(self, '_colmap_f'):
            self._colmap_f.close()
        self.get_logger().info(
            f'⏹ Stopped. {self.cloud_count} clouds, '
            f'{self.image_count} images saved → {self.output_dir}')
 
    def _open_csv(self, name, header):
        fh = open(self.output_dir / f'{name}.csv', 'w',
                  newline='', encoding='utf-8')
        w  = csv.writer(fh)
        w.writerow(header)
        self.csv_files[name]   = fh
        self.csv_writers[name] = w
 
    def _ts(self, stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9
 
    # ── camera info (save once) ───────────────────────────────────
 
    def cam_info_cb(self, msg: CameraInfo):
        if not self.recording or self._cam_info_saved:
            return
        info = {
            'width':  msg.width,
            'height': msg.height,
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5],
            'distortion_model': msg.distortion_model,
            'D': list(msg.d),
        }
        with open(self.output_dir / 'camera_info.json', 'w') as f:
            json.dump(info, f, indent=2)
        self._cam_info_saved = True
        self.get_logger().info(
            f'Camera intrinsics saved: {msg.width}×{msg.height} '
            f'fx={info["fx"]:.1f} fy={info["fy"]:.1f}')
 
    # ── RGB image cache ───────────────────────────────────────────
 
    def image_cb(self, msg: Image):
        self._latest_image = msg   # just cache — written with each cloud
 
    # ── point cloud ───────────────────────────────────────────────
 
    def cloud_cb(self, msg: PointCloud2):
        if not self.recording:
            return
        self.cloud_count += 1
        if self.cloud_count % self.cloud_every_n != 0:
            return
 
        # extract XYZ + colour
        fields = [f.name for f in msg.fields]
        has_rgb = 'rgb' in fields
        read_fields = ['x', 'y', 'z'] + (['rgb'] if has_rgb else [])
 
        pts, cols = [], []
        for pt in point_cloud2.read_points(
                msg, field_names=read_fields, skip_nans=True):
            pts.append((float(pt[0]), float(pt[1]), float(pt[2])))
            if has_rgb:
                cols.append(decode_rgb(float(pt[3])))
 
        if not pts:
            return
 
        idx = self.frame_idx
        self.frame_idx += 1
 
        # save PLY
        ply_name = f'cloud_{idx:06d}.ply'
        write_ply_binary(
            self.output_dir / 'pointclouds' / ply_name,
            pts, cols if has_rgb else None)
 
        # save matched RGB image as raw bytes (encoding = rgb8 / bgr8)
        if self._latest_image is not None:
            img_name = f'frame_{idx:06d}.jpg'
            img_path = self.output_dir / 'images' / img_name
            self._save_image(self._latest_image, img_path)
            self.image_count += 1
 
        # look up sensor→odom transform and write to transforms CSV + COLMAP
        ts = self._ts(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, msg.header.stamp,
                rclpy.duration.Duration(seconds=0.15))
            t = tf.transform.translation
            r = tf.transform.rotation
            self.csv_writers['transforms'].writerow([
                f'{ts:.6f}', 'odom', msg.header.frame_id,
                f'{t.x:.6f}', f'{t.y:.6f}', f'{t.z:.6f}',
                f'{r.x:.6f}', f'{r.y:.6f}', f'{r.z:.6f}', f'{r.w:.6f}'])
 
            # COLMAP images.txt line
            qw, qx, qy, qz, tx, ty, tz = pose_to_colmap_extrinsic(
                t.x, t.y, t.z, r.x, r.y, r.z, r.w)
            self._colmap_f.write(
                f'{idx} {qw:.8f} {qx:.8f} {qy:.8f} {qz:.8f} '
                f'{tx:.6f} {ty:.6f} {tz:.6f} 1 frame_{idx:06d}.jpg\n\n')
            self._colmap_f.flush()
 
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}',
                                   throttle_duration_sec=5.0)
 
        self.get_logger().info(
            f'Saved cloud_{idx:06d} ({len(pts)} pts)',
            throttle_duration_sec=2.0)
 
    def _save_image(self, msg: Image, path: Path):
        """Save raw ROS image to JPEG using only stdlib (no cv_bridge needed)."""
        # Try cv_bridge first; fall back to raw PPM→JPEG-less save
        try:
            import cv2
            import numpy as np
            enc = msg.encoding.lower()
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if 'bgr' in enc:
                cv2.imwrite(str(path), arr)
            else:
                cv2.imwrite(str(path), cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
        except ImportError:
            # No cv2: save as raw binary (rename .jpg → .raw in this case)
            raw_path = path.with_suffix('.raw')
            raw_path.write_bytes(bytes(msg.data))
 
    # ── IMU ───────────────────────────────────────────────────────
 
    def imu_cb(self, msg: Imu):
        if not self.recording:
            return
        ts = self._ts(msg.header.stamp)
        a, w, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        self.csv_writers['imu'].writerow([
            f'{ts:.6f}',
            f'{a.x:.6f}', f'{a.y:.6f}', f'{a.z:.6f}',
            f'{w.x:.6f}', f'{w.y:.6f}', f'{w.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}'])
 
    # ── odometry ─────────────────────────────────────────────────
 
    def odom_cb(self, msg: Odometry):
        if not self.recording:
            return
        ts = self._ts(msg.header.stamp)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        row = [f'{ts:.6f}',
               f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
               f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}',
               f'{v.x:.6f}', f'{v.y:.6f}', f'{v.z:.6f}',
               f'{w.x:.6f}', f'{w.y:.6f}', f'{w.z:.6f}']
        self.csv_writers['odometry'].writerow(row)
        self.csv_writers['trajectory'].writerow(row[:8])
 
    # ── ground truth ─────────────────────────────────────────────
 
    def gt_cb(self, msg: PoseArray):
        if not self.recording or not msg.poses:
            return
        ts = self._ts(msg.header.stamp)
        # Find romi by index once (it's the only dynamic model)
        if self._romi_idx < 0:
            # Romi spawns at ~(1,1) world — pick closest on first message
            best, best_d = -1, 1.0
            for i, pose in enumerate(msg.poses):
                d = math.hypot(pose.position.x - 1.0, pose.position.y - 1.0)
                if d < best_d:
                    best_d, best = d, i
            self._romi_idx = max(best, 0)
 
        if self._romi_idx >= len(msg.poses):
            return
        pose = msg.poses[self._romi_idx]
        p, q = pose.position, pose.orientation
        self.csv_writers['ground_truth'].writerow([
            f'{ts:.6f}',
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}'])
 
 
def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.recording:
            node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
