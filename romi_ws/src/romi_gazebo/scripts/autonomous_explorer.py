#!/usr/bin/env python3
 
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
 
 
# ── tiny grid for coverage tracking ──────────────────────────────
class CoverageGrid:
    def __init__(self, cell_size: float = 0.5):
        self.cell  = cell_size
        self.visited: set = set()
 
    def mark(self, x: float, y: float):
        self.visited.add((int(x / self.cell), int(y / self.cell)))
 
    def count(self) -> int:
        return len(self.visited)
 
    def score_direction(self, x: float, y: float, yaw: float,
                        heading_offset: float, lookahead: float = 3.0) -> int:
        """Count unvisited cells in a cone ahead of (x,y) at yaw+heading_offset."""
        angle  = yaw + heading_offset
        steps  = int(lookahead / self.cell)
        unseen = 0
        for s in range(1, steps + 1):
            cx = int((x + s * self.cell * math.cos(angle)) / self.cell)
            cy = int((y + s * self.cell * math.sin(angle)) / self.cell)
            if (cx, cy) not in self.visited:
                unseen += 1
        return unseen
 
 
class State:
    WAITING    = 'WAITING'
    EXPLORING  = 'EXPLORING'
    TURNING    = 'TURNING'
    REVERSING  = 'REVERSING'
    RECOVERING = 'RECOVERING'
    DONE       = 'DONE'
 
 
class ReactiveExplorer(Node):
    def __init__(self):
        super().__init__('reactive_explorer')
 
        # ── parameters ───────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic',        '/model/romi/cmd_vel')
        self.declare_parameter('scan_topic',           '/lidar/scan')
        self.declare_parameter('odom_topic',           '/model/romi/odometry')
        self.declare_parameter('linear_speed',          0.22)
        self.declare_parameter('angular_speed',         0.55)
        self.declare_parameter('obstacle_threshold',    1.2)
        self.declare_parameter('side_threshold',        0.45)
        self.declare_parameter('turn_duration',         2.8)
        # Coverage: stop when this many unique cells visited
        # Warehouse is ~30×15 m = 450 m² → ~1800 cells at 0.5 m
        # 600 cells ≈ 33 % coverage — enough for full 3D dataset
        self.declare_parameter('coverage_stop_cells',   600)
        # Fallback time limit in seconds (0 = no limit)
        self.declare_parameter('exploration_timeout',   0.0)
        # How far robot must move before stuck counter resets (metres)
        self.declare_parameter('stuck_move_threshold',  0.06)
        # After this many control ticks without movement → recovery
        self.declare_parameter('stuck_ticks',           60)   # 6 s at 10 Hz
        self.declare_parameter('coverage_cell_size',    0.5)
 
        self.linear_speed       = self.get_parameter('linear_speed').value
        self.angular_speed      = self.get_parameter('angular_speed').value
        self.obs_thr            = self.get_parameter('obstacle_threshold').value
        self.side_thr           = self.get_parameter('side_threshold').value
        self.turn_dur           = self.get_parameter('turn_duration').value
        self.coverage_stop      = self.get_parameter('coverage_stop_cells').value
        self.exp_timeout        = self.get_parameter('exploration_timeout').value
        self.stuck_move_thr     = self.get_parameter('stuck_move_threshold').value
        self.stuck_ticks_limit  = self.get_parameter('stuck_ticks').value
 
        # ── pub / sub ────────────────────────────────────────────
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub    = self.create_publisher(Twist,  cmd_topic, 10)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
 
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
 
        # ── state ────────────────────────────────────────────────
        self.state          = State.WAITING
        self.latest_scan    = None
        self.recording_on   = False
        self.state_timer    = 0.0
        self.turn_dir       = 1.0
        self._wall_hz       = 0.0     # wall-follow hysteresis
        self.timer_period   = 0.1
 
        # pose from odometry (used for coverage + stuck detection)
        self.rx = 0.0
        self.ry = 0.0
        self.ryaw = 0.0
 
        # stuck detection (tick-based so sim-time pauses don't trigger it)
        self._last_x      = None
        self._last_y      = None
        self._stuck_ticks = 0
 
        # exploration timeout (wall-clock seconds since spin start)
        self._start_sec   = None
 
        # coverage tracking
        self.grid = CoverageGrid(
            self.get_parameter('coverage_cell_size').value)
 
        self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info('ReactiveExplorer (final) ready — waiting for LiDAR...')
 
    # ── helpers ──────────────────────────────────────────────────
 
    def sector_min(self, scan: LaserScan,
                   deg_start: float, deg_end: float) -> float:
        a0 = math.radians(deg_start)
        a1 = math.radians(deg_end)
        i0 = max(0, int((a0 - scan.angle_min) / scan.angle_increment))
        i1 = min(len(scan.ranges)-1,
                 int((a1 - scan.angle_min) / scan.angle_increment))
        if i0 > i1:
            i0, i1 = i1, i0
        vals = [r for r in scan.ranges[i0:i1+1]
                if math.isfinite(r)
                and scan.range_min < r < scan.range_max]
        return min(vals) if vals else float('inf')
 
    def _toggle_recording(self, on: bool):
        if on == self.recording_on:
            return
        cli = self.create_client(SetBool, 'toggle_recording')
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('toggle_recording service not available')
            return
        req      = SetBool.Request()
        req.data = on
        fut = cli.call_async(req)
        fut.add_done_callback(
            lambda f: self.get_logger().info(
                f'Recorder: {f.result().message}'
                if f.result() else 'Recorder call failed'))
        self.recording_on = on
 
    def _save_map_and_stop(self):
        """Stop robot, stop recording, save map, then shut the node down."""
        self.get_logger().info(
            f'Coverage goal reached ({self.grid.count()} cells) — '
            'stopping, saving map and shutting down.')
        self.cmd_pub.publish(Twist())          # zero velocity
        self._toggle_recording(False)           # flush CSV + PLY files
 
        # Save the occupancy grid map
        import subprocess, time as _t
        ts       = _t.strftime('%Y%m%d_%H%M%S')
        map_path = f'/tmp/romi_map_{ts}'
        self.get_logger().info(f'Saving map → {map_path}.pgm / .yaml')
        subprocess.Popen([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_path,
            '--ros-args', '-p', 'use_sim_time:=true'
        ])
 
        self.state = State.DONE
        # Schedule node shutdown after 3 s (gives map_saver time to write)
        self.create_timer(3.0, self._shutdown)
 
    def _shutdown(self):
        self.get_logger().info('Explorer shutting down cleanly.')
        raise ExternalShutdownException()
 
    # ── callbacks ────────────────────────────────────────────────
 
    def scan_cb(self, msg: LaserScan):
        if self.state == State.WAITING:
            self.get_logger().info('First LiDAR scan — starting exploration.')
            self.state    = State.EXPLORING
            self._toggle_recording(True)
        self.latest_scan = msg
 
    def odom_cb(self, msg: Odometry):
        p        = msg.pose.pose.position
        q        = msg.pose.pose.orientation
        self.rx  = p.x
        self.ry  = p.y
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.ryaw = math.atan2(siny, cosy)
 
        # mark visited cell
        self.grid.mark(self.rx, self.ry)
 
        # stuck detection
        if self._last_x is None:
            self._last_x, self._last_y = self.rx, self.ry
        d = math.hypot(self.rx - self._last_x, self.ry - self._last_y)
        if d > self.stuck_move_thr:
            self._last_x, self._last_y = self.rx, self.ry
            self._stuck_ticks = 0
        else:
            self._stuck_ticks += 1
 
    # ── main control loop ────────────────────────────────────────
 
    def control_loop(self):
        if self.state in (State.WAITING, State.DONE) \
                or self.latest_scan is None:
            return
 
        # ── timeout check ─────────────────────────────────────────
        import time as _t
        if self._start_sec is None:
            self._start_sec = _t.monotonic()
        if self.exp_timeout > 0:
            if _t.monotonic() - self._start_sec > self.exp_timeout:
                self._save_map_and_stop()
                return
 
        # ── coverage check ────────────────────────────────────────
        if self.grid.count() >= self.coverage_stop:
            self._save_map_and_stop()
            return
 
        scan  = self.latest_scan
        twist = Twist()
 
        # sensor zones
        front  = self.sector_min(scan, -20,   20)
        fl     = self.sector_min(scan,  20,   60)
        fr     = self.sector_min(scan, -60,  -20)
        left   = self.sector_min(scan,  60,  120)
        right  = self.sector_min(scan, -120, -60)
        thr    = self.obs_thr
 
        # ── emergency collision guard ─────────────────────────────
        if min(front, fl, fr) < 0.22 \
                and self.state not in (State.REVERSING, State.RECOVERING):
            self.state       = State.REVERSING
            self.state_timer = 1.8
            self.turn_dir    = 1.0 if fl < fr else -1.0
            self.get_logger().warn(
                f'Collision imminent ({min(front,fl,fr):.2f} m) — reversing')
 
        # ── stuck guard ───────────────────────────────────────────
        if self._stuck_ticks >= self.stuck_ticks_limit \
                and self.state == State.EXPLORING:
            self.state        = State.RECOVERING
            self.state_timer  = 3.5
            self.turn_dir     = 1.0 if right < left else -1.0
            self._stuck_ticks = 0
            self.get_logger().warn('Stuck — forcing recovery spin')
 
        # ── state machine ─────────────────────────────────────────
 
        if self.state == State.EXPLORING:
            obstacle_ahead = (front < thr
                              or fl < thr * 0.7
                              or fr < thr * 0.7)
 
            if obstacle_ahead:
                # score four candidate directions using coverage grid
                candidates = [
                    ( math.pi / 2,  left),        # hard left
                    ( math.pi / 4,  fl),           # soft left
                    (-math.pi / 4,  fr),           # soft right
                    (-math.pi / 2,  right),        # hard right
                ]
                best_score = -1
                best_dir   = 1.0
                for offset, clearance in candidates:
                    if clearance < 0.35:           # skip directions that are blocked
                        continue
                    unseen = self.grid.score_direction(
                        self.rx, self.ry, self.ryaw, offset)
                    if unseen > best_score:
                        best_score = unseen
                        best_dir   = 1.0 if offset >= 0 else -1.0
 
                self.turn_dir    = best_dir
                self.state       = State.TURNING
                jitter           = 0.7 + 0.6 * random.random()
                self.state_timer = self.turn_dur * jitter
                side = 'LEFT' if best_dir > 0 else 'RIGHT'
                self.get_logger().info(
                    f'Obstacle {front:.2f} m — turning {side} '
                    f'(coverage score {best_score})')
            else:
                # forward movement + wall-follow hysteresis
                twist.linear.x = self.linear_speed
                if left < self.side_thr:
                    self._wall_hz = -0.22
                elif right < self.side_thr:
                    self._wall_hz = +0.22
                else:
                    self._wall_hz *= 0.88
                    if abs(self._wall_hz) < 0.01:
                        self._wall_hz = 0.0
                noise           = random.uniform(-0.06, 0.06)
                twist.angular.z = self._wall_hz + noise
 
        elif self.state == State.TURNING:
            self.state_timer -= self.timer_period
            twist.angular.z   = self.angular_speed * self.turn_dir
            clear = (front > thr * 1.3
                     and fl > thr * 0.9
                     and fr > thr * 0.9)
            if self.state_timer <= 0 or clear:
                self.state = State.EXPLORING
 
        elif self.state == State.REVERSING:
            self.state_timer  -= self.timer_period
            twist.linear.x     = -self.linear_speed * 0.75
            twist.angular.z    = self.angular_speed * 0.5 * self.turn_dir
            if self.state_timer <= 0:
                self.state       = State.TURNING
                self.state_timer = self.turn_dur * 1.2
 
        elif self.state == State.RECOVERING:
            self.state_timer -= self.timer_period
            twist.angular.z   = self.angular_speed * self.turn_dir
            if self.state_timer <= 0 or front > thr * 1.2:
                self.state = State.EXPLORING
 
        self.cmd_pub.publish(twist)
 
        # status
        msg        = String()
        msg.data   = (f'state={self.state} '
                      f'cells={self.grid.count()}/{self.coverage_stop} '
                      f'f={front:.2f} l={left:.2f} r={right:.2f}')
        self.status_pub.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ReactiveExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
 
 
if __name__ == '__main__':
    main()
