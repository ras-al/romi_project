#!/usr/bin/env python3
"""
data_recorder.py  — Comprehensive 3D reconstruction dataset recorder
─────────────────────────────────────────────────────────────────────
Captures:
  • RGB images      → images/rgb/frame_XXXXXX.jpg
  • Depth images    → images/depth/frame_XXXXXX.png  (16-bit PNG, millimetres)
  • Point clouds    → pointclouds/cloud_XXXXXX.ply   (binary little-endian)
  • Odometry        → odometry.csv                   (filtered EKF output)
  • Raw odometry    → odometry_raw.csv               (direct wheel encoder)
  • IMU             → imu.csv                        (100 Hz accel + gyro + quat)
  • TF transforms   → transforms.csv                 (every sensor→odom lookup)
  • Ground truth    → ground_truth.csv               (Gazebo world pose)
  • Camera info     → camera_info.json               (intrinsics, one-shot)
  • COLMAP poses    → images.txt                     (ready for SfM pipeline)
  • Trajectory      → trajectory.csv                 (x,y,z,qx,qy,qz,qw)
"""

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

def pose_to_colmap_extrinsic(tx, ty, tz, qx, qy, qz, qw):
    """Return (qw, qx, qy, qz, tx, ty, tz) for COLMAP images.txt."""
    return qw, qx, qy, qz, tx, ty, tz


# ─────────────────────────────────────────────────────────────────

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        self.declare_parameter('output_dir',        '')
        self.declare_parameter('cloud_topic',       '/depth_camera/points')
        self.declare_parameter('image_topic',       '/depth_camera/image')
        self.declare_parameter('depth_topic',       '/depth_camera/depth_image')
        self.declare_parameter('cam_info_topic',    '/depth_camera/camera_info')
        self.declare_parameter('imu_topic',         '/imu')
        self.declare_parameter('odom_topic',        '/odometry/filtered')
        self.declare_parameter('raw_odom_topic',    '/model/romi/odometry')
        self.declare_parameter('pose_topic',        '/world/world_demo/dynamic_pose/info')
        self.declare_parameter('cloud_every_n',     3)
        self.declare_parameter('auto_start',        True)

        self.cloud_topic     = self.get_parameter('cloud_topic').value
        self.image_topic     = self.get_parameter('image_topic').value
        self.depth_topic     = self.get_parameter('depth_topic').value
        self.cam_info_topic  = self.get_parameter('cam_info_topic').value
        self.imu_topic       = self.get_parameter('imu_topic').value
        self.odom_topic      = self.get_parameter('odom_topic').value
        self.raw_odom_topic  = self.get_parameter('raw_odom_topic').value
        self.pose_topic      = self.get_parameter('pose_topic').value
        self.cloud_every_n   = max(1, int(self.get_parameter('cloud_every_n').value))
        auto_start           = self.get_parameter('auto_start').value

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.recording     = False
        self.output_dir    = None
        self.cloud_count   = 0
        self.rgb_count     = 0
        self.depth_count   = 0
        self.frame_idx     = 0
        self.csv_files     = {}
        self.csv_writers   = {}
        self._cam_info_saved = False
        self._latest_image   = None
        self._latest_depth   = None
        self._romi_idx       = -1

        # Service
        self.srv = self.create_service(
            SetBool, 'toggle_recording', self.toggle_cb)

        # Subscriptions
        self.create_subscription(PointCloud2, self.cloud_topic,     self.cloud_cb,     10)
        self.create_subscription(Image,       self.image_topic,     self.image_cb,     10)
        self.create_subscription(Image,       self.depth_topic,     self.depth_cb,     10)
        self.create_subscription(CameraInfo,  self.cam_info_topic,  self.cam_info_cb,  10)
        self.create_subscription(Imu,         self.imu_topic,       self.imu_cb,       10)
        self.create_subscription(Odometry,    self.odom_topic,      self.odom_cb,      10)
        self.create_subscription(Odometry,    self.raw_odom_topic,  self.raw_odom_cb,  10)
        self.create_subscription(PoseArray,   self.pose_topic,      self.gt_cb,        10)

        if auto_start:
            self._start()
        else:
            self.get_logger().info(
                'DataRecorder ready — call /toggle_recording (true) to start')

    # ── toggle service ───────────────────────────────────────────
    # PAUSE/RESUME model:
    #   First True  → _init_session() creates folder + files (once)
    #   Subsequent True  → resume  (just set recording=True)
    #   False        → pause  (set recording=False, files stay OPEN)
    # Files are only closed when the node shuts down.

    def toggle_cb(self, req, res):
        if req.data and not self.recording:
            if self.output_dir is None:
                # First ever start — create the session
                self._init_session()
            else:
                # Resume after pause — same folder, same files
                self.recording = True
                self.get_logger().info(f'▶ Resumed → {self.output_dir}')
            res.success = True
            res.message = f'Recording → {self.output_dir}'
        elif not req.data and self.recording:
            # Pause only — do NOT close files or create a new folder later
            self.recording = False
            self.get_logger().info(
                f'⏸ Paused ({self.cloud_count} clouds, '
                f'{self.rgb_count} RGB so far)')
            res.success = True
            res.message = 'Paused.'
        else:
            res.success = True
            res.message = 'Already in requested state.'
        return res

    # ── session lifecycle ─────────────────────────────────────────

    def _init_session(self):
        """Create the session folder and open all files exactly ONCE."""
        p = self.get_parameter('output_dir').value
        if p:
            base = Path(p)
        else:
            # Use absolute path based on workspace location
            ws = Path(__file__).resolve().parent.parent.parent.parent
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            base = ws / 'data' / f'romi_capture_{ts}'

        # Never overwrite an existing session — append suffix if needed
        candidate = base
        suffix = 1
        while candidate.exists():
            candidate = Path(f'{base}_{suffix}')
            suffix += 1
        self.output_dir = candidate

        (self.output_dir / 'pointclouds').mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'images' / 'rgb').mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'images' / 'depth').mkdir(parents=True, exist_ok=True)

        self._open_csv('trajectory',   ['timestamp', 'x', 'y', 'z',
                                         'qx', 'qy', 'qz', 'qw'])
        self._open_csv('imu',          ['timestamp',
                                         'ax', 'ay', 'az',
                                         'wx', 'wy', 'wz',
                                         'ox', 'oy', 'oz', 'ow'])
        self._open_csv('odometry',     ['timestamp', 'x', 'y', 'z',
                                         'qx', 'qy', 'qz', 'qw',
                                         'vx', 'vy', 'vz',
                                         'wx', 'wy', 'wz'])
        self._open_csv('odometry_raw', ['timestamp', 'x', 'y', 'z',
                                         'qx', 'qy', 'qz', 'qw',
                                         'vx', 'vy', 'vz',
                                         'wx', 'wy', 'wz'])
        self._open_csv('ground_truth', ['timestamp', 'x', 'y', 'z',
                                         'qx', 'qy', 'qz', 'qw'])
        self._open_csv('transforms',   ['timestamp', 'parent_frame',
                                         'child_frame', 'tx', 'ty', 'tz',
                                         'qx', 'qy', 'qz', 'qw'])

        # COLMAP images.txt (header written once)
        self._colmap_f = open(
            self.output_dir / 'images.txt', 'w', encoding='utf-8')
        self._colmap_f.write(
            '# COLMAP images.txt  IMAGE_ID QW QX QY QZ TX TY TZ '
            'CAMERA_ID NAME\n# (one blank line per image, no 2D points)\n')

        # Do NOT reset cloud_count/frame_idx here — they accumulate
        # across pause/resume cycles within the same session.
        self.recording       = True
        self._cam_info_saved = False
        self.get_logger().info(f'▶ Recording → {self.output_dir}')

    # _start is kept as an alias so auto_start=True still works
    def _start(self):
        if self.output_dir is None:
            self._init_session()
        else:
            self.recording = True

    def _stop(self):
        """Pause recording (do not close files)."""
        self.recording = False
        self.get_logger().info(
            f'⏸ Paused. {self.cloud_count} clouds, '
            f'{self.rgb_count} RGB, {self.depth_count} depth so far '
            f'→ {self.output_dir}')

    def _close_session(self):
        """Flush and close all files — called only on node shutdown."""
        self.recording = False
        for fh in self.csv_files.values():
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        self.csv_files.clear()
        self.csv_writers.clear()
        if hasattr(self, '_colmap_f') and self._colmap_f \
                and not self._colmap_f.closed:
            self._colmap_f.flush()
            self._colmap_f.close()
        self.get_logger().info(
            f'⏹ Session closed. {self.cloud_count} clouds, '
            f'{self.rgb_count} RGB, {self.depth_count} depth '
            f'→ {self.output_dir}')

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
            'K': list(msg.k),
            'distortion_model': msg.distortion_model,
            'D': list(msg.d),
            'R': list(msg.r),
            'P': list(msg.p),
        }
        with open(self.output_dir / 'camera_info.json', 'w') as f:
            json.dump(info, f, indent=2)
        self._cam_info_saved = True
        self.get_logger().info(
            f'Camera intrinsics saved: {msg.width}×{msg.height} '
            f'fx={info["fx"]:.1f} fy={info["fy"]:.1f}')

    # ── RGB image cache ───────────────────────────────────────────

    def image_cb(self, msg: Image):
        self._latest_image = msg

    # ── Depth image cache ─────────────────────────────────────────

    def depth_cb(self, msg: Image):
        self._latest_depth = msg

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

        # save matched RGB image
        if self._latest_image is not None:
            img_name = f'frame_{idx:06d}.jpg'
            img_path = self.output_dir / 'images' / 'rgb' / img_name
            self._save_rgb(self._latest_image, img_path)
            self.rgb_count += 1

        # save matched depth image as 16-bit PNG (millimetres)
        if self._latest_depth is not None:
            depth_name = f'frame_{idx:06d}.png'
            depth_path = self.output_dir / 'images' / 'depth' / depth_name
            self._save_depth(self._latest_depth, depth_path)
            self.depth_count += 1

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

        # Also record additional TF pairs we care about
        self._record_tf('odom', 'base_link', msg.header.stamp, ts)
        self._record_tf('base_link', 'romi/sensor_mount', msg.header.stamp, ts)
        self._record_tf('map', 'odom', msg.header.stamp, ts)

        self.get_logger().info(
            f'Saved frame_{idx:06d} ({len(pts)} pts, '
            f'RGB={"✓" if self._latest_image else "✗"}, '
            f'depth={"✓" if self._latest_depth else "✗"})',
            throttle_duration_sec=2.0)

    def _record_tf(self, parent, child, stamp, ts):
        """Try to record a TF pair, silently skip if unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                parent, child, stamp,
                rclpy.duration.Duration(seconds=0.05))
            t = tf.transform.translation
            r = tf.transform.rotation
            self.csv_writers['transforms'].writerow([
                f'{ts:.6f}', parent, child,
                f'{t.x:.6f}', f'{t.y:.6f}', f'{t.z:.6f}',
                f'{r.x:.6f}', f'{r.y:.6f}', f'{r.z:.6f}', f'{r.w:.6f}'])
        except Exception:
            pass

    def _save_rgb(self, msg: Image, path: Path):
        """Save ROS Image as JPEG."""
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
            raw_path = path.with_suffix('.raw')
            raw_path.write_bytes(bytes(msg.data))

    def _save_depth(self, msg: Image, path: Path):
        """Save depth image as 16-bit PNG (millimetres).

        Gazebo RGBD camera publishes 32FC1 (metres as float32).
        We convert to uint16 millimetres for standard depth map format
        used by Open3D, BundleFusion, etc.
        """
        try:
            import cv2
            import numpy as np
            enc = msg.encoding.lower()
            if '32fc1' in enc:
                # 32-bit float metres → 16-bit uint millimetres
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    msg.height, msg.width)
                arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
                depth_mm = (arr * 1000.0).clip(0, 65535).astype(np.uint16)
                cv2.imwrite(str(path), depth_mm)
            elif '16uc1' in enc:
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width)
                cv2.imwrite(str(path), arr)
            else:
                # Unknown encoding — save raw
                raw_path = path.with_suffix('.raw')
                raw_path.write_bytes(bytes(msg.data))
                self.get_logger().warn(
                    f'Unknown depth encoding: {msg.encoding}',
                    throttle_duration_sec=10.0)
        except ImportError:
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

    # ── filtered odometry ────────────────────────────────────────

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

    # ── raw wheel odometry ────────────────────────────────────────

    def raw_odom_cb(self, msg: Odometry):
        if not self.recording:
            return
        ts = self._ts(msg.header.stamp)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.csv_writers['odometry_raw'].writerow([
            f'{ts:.6f}',
            f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}',
            f'{v.x:.6f}', f'{v.y:.6f}', f'{v.z:.6f}',
            f'{w.x:.6f}', f'{w.y:.6f}', f'{w.z:.6f}'])

    # ── ground truth ─────────────────────────────────────────────

    def gt_cb(self, msg: PoseArray):
        if not self.recording or not msg.poses:
            return
        ts = self._ts(msg.header.stamp)
        if self._romi_idx < 0:
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
        node._close_session()   # flush + close all files, even if paused
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
