#!/usr/bin/env python3
"""
Autonomous reactive explorer for the Romi robot.

Implements coverage-based frontier exploration with aggressive
scan-driven obstacle avoidance and collision recovery.
"""

import math
import random
import time as _time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


# Coverage grid

class CoverageGrid:
    """Discretised 2D grid tracking visited cells."""

    def __init__(self, cell_size: float = 0.5):
        self.cell = cell_size
        self.visited: set = set()

    def mark(self, x: float, y: float):
        self.visited.add((int(x / self.cell), int(y / self.cell)))

    def count(self) -> int:
        return len(self.visited)

    def score_direction(self, x: float, y: float, yaw: float,
                        offset: float, lookahead: float = 5.0) -> int:
        """Score a heading by counting unvisited cells along it."""
        angle = yaw + offset
        steps = int(lookahead / self.cell)
        score = 0
        for s in range(1, steps + 1):
            cx = int((x + s * self.cell * math.cos(angle)) / self.cell)
            cy = int((y + s * self.cell * math.sin(angle)) / self.cell)
            if (cx, cy) not in self.visited:
                score += max(1, steps - s + 1)
        return score


# Finite-state-machine states

class St:
    WAITING    = 'WAITING'
    EXPLORING  = 'EXPLORING'
    AVOIDING   = 'AVOIDING'     # In-place rotation away from obstacle
    REVERSING  = 'REVERSING'    # Emergency collision reverse
    RECOVERING = 'RECOVERING'   # Recovery from stuck state
    DONE       = 'DONE'


class ReactiveExplorer(Node):

    def __init__(self):
        super().__init__('reactive_explorer')

        # ROS parameters
        self.declare_parameter('cmd_vel_topic',       '/model/romi/cmd_vel')
        self.declare_parameter('scan_topic',          '/lidar/scan')
        self.declare_parameter('odom_topic',          '/model/romi/odometry')
        self.declare_parameter('linear_speed',         0.20)
        self.declare_parameter('angular_speed',        1.0)
        self.declare_parameter('obstacle_threshold',   0.60)
        self.declare_parameter('emergency_threshold',  0.30)
        self.declare_parameter('side_threshold',       0.35)
        self.declare_parameter('coverage_stop_cells',  600)
        self.declare_parameter('exploration_timeout',  0.0)
        self.declare_parameter('stuck_ticks',          30)
        self.declare_parameter('stuck_move_threshold', 0.03)
        self.declare_parameter('coverage_cell_size',   0.5)
        self.declare_parameter('progress_watchdog_s',  60.0)

        self.v_lin     = self.get_parameter('linear_speed').value
        self.v_ang     = self.get_parameter('angular_speed').value
        self.obs_thr   = self.get_parameter('obstacle_threshold').value
        self.emg_thr   = self.get_parameter('emergency_threshold').value
        self.side_thr  = self.get_parameter('side_threshold').value
        self.cov_stop  = self.get_parameter('coverage_stop_cells').value
        self.exp_to    = self.get_parameter('exploration_timeout').value
        self.stuck_lim = self.get_parameter('stuck_ticks').value
        self.stuck_mov = self.get_parameter('stuck_move_threshold').value
        self.prog_wd   = self.get_parameter('progress_watchdog_s').value

        # Publishers / subscribers
        cmd = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub    = self.create_publisher(Twist,  cmd, 10)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)

        # State
        self.state       = St.WAITING
        self.latest_scan = None
        self.rec_on      = False
        self.DT          = 0.05
        self.turn_dir    = 1.0
        self._wall_hz    = 0.0
        self._avoid_timer = 0.0
        self._rev_timer   = 0.0

        # Pose
        self.rx = self.ry = self.ryaw = 0.0

        # Stuck detection
        self._last_x    = self._last_y = None
        self._stk_ticks = 0

        # Coverage
        self.grid = CoverageGrid(self.get_parameter('coverage_cell_size').value)

        # Progress watchdog
        self._last_cov_count = 0
        self._last_cov_time  = _time.monotonic()

        # Timeout
        self._t0 = None

        self.create_timer(self.DT, self.loop)
        self.get_logger().info(
            f'Explorer ready  obs={self.obs_thr:.2f}m  '
            f'emg={self.emg_thr:.2f}m  ang={self.v_ang:.1f}')

    # Scan helpers

    def zone(self, scan: LaserScan, d0: float, d1: float) -> float:
        """Minimum valid range in degree arc [d0, d1]. 0° = forward."""
        inc  = scan.angle_increment
        amin = scan.angle_min
        i0 = max(0, int((math.radians(d0) - amin) / inc))
        i1 = min(len(scan.ranges) - 1, int((math.radians(d1) - amin) / inc))
        if i0 > i1:
            i0, i1 = i1, i0
        v = [r for r in scan.ranges[i0:i1 + 1]
             if math.isfinite(r) and scan.range_min < r < scan.range_max]
        return min(v) if v else float('inf')

    def _pick_clear_direction(self, scan) -> float:
        """Choose turn direction toward the clearest, most unexplored space.

        Returns +1.0 (CCW / left) or -1.0 (CW / right).
        """
        # Check 10 candidate headings from 30° to 150° on both sides
        candidates = []
        for deg in [30, 60, 90, 120, 150]:
            for sign, direction in [(1, 1.0), (-1, -1.0)]:
                offset_rad = sign * math.radians(deg)
                centre = sign * deg
                clearance = self.zone(scan, centre - 20, centre + 20)
                if clearance < 0.35:
                    continue
                cov = self.grid.score_direction(
                    self.rx, self.ry, self.ryaw, offset_rad)
                score = cov + clearance * 15.0
                candidates.append((score, direction))

        if candidates:
            candidates.sort(reverse=True)
            return candidates[0][1]

        # Fallback: turn away from the closer side
        l = self.zone(scan, 60, 120)
        r = self.zone(scan, -120, -60)
        return 1.0 if r <= l else -1.0

    # Recording toggle

    def _toggle_rec(self, on: bool):
        if on == self.rec_on:
            return
        cli = self.create_client(SetBool, 'toggle_recording')
        if not cli.wait_for_service(timeout_sec=1.5):
            return
        req = SetBool.Request()
        req.data = on
        cli.call_async(req)
        self.rec_on = on

    # Finish exploration

    def _finish(self):
        self.get_logger().info(
            f'Exploration complete — {self.grid.count()} cells. Saving map...')
        self.cmd_pub.publish(Twist())
        self._toggle_rec(False)
        import subprocess
        ts = _time.strftime('%Y%m%d_%H%M%S')
        subprocess.Popen([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', f'/tmp/romi_map_{ts}',
            '--ros-args', '-p', 'use_sim_time:=true'])
        self.state = St.DONE
        self.create_timer(
            3.5, lambda: (_ for _ in ()).throw(ExternalShutdownException()))

    # Callbacks

    def scan_cb(self, msg: LaserScan):
        if self.state == St.WAITING:
            self.get_logger().info('LiDAR received — starting exploration')
            self.state = St.EXPLORING
            self._toggle_rec(True)
        self.latest_scan = msg

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rx, self.ry = p.x, p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        self.ryaw = math.atan2(siny, cosy)
        self.grid.mark(self.rx, self.ry)

        # Stuck detection
        if self._last_x is None:
            self._last_x, self._last_y = self.rx, self.ry
        d = math.hypot(self.rx - self._last_x, self.ry - self._last_y)
        if d > self.stuck_mov:
            self._last_x, self._last_y = self.rx, self.ry
            self._stk_ticks = 0
        else:
            self._stk_ticks += 1

    # Main control loop (20 Hz)

    def loop(self):
        if self.state in (St.WAITING, St.DONE) or self.latest_scan is None:
            return

        # Termination
        if self._t0 is None:
            self._t0 = _time.monotonic()
        now = _time.monotonic()

        if self.exp_to > 0 and (now - self._t0) > self.exp_to:
            self._finish(); return
        if self.grid.count() >= self.cov_stop:
            self._finish(); return

        # Progress watchdog
        cur_cov = self.grid.count()
        if cur_cov > self._last_cov_count:
            self._last_cov_count = cur_cov
            self._last_cov_time  = now
        elif (now - self._last_cov_time) > self.prog_wd \
                and self.state == St.EXPLORING:
            self.get_logger().warn('Coverage stalled — forcing new heading')
            self.turn_dir       = random.choice([1.0, -1.0])
            self.state          = St.AVOIDING
            self._avoid_timer   = 2.0
            self._last_cov_time = now

        scan  = self.latest_scan
        twist = Twist()

        # Read all sensor zones
        front = self.zone(scan, -25,  25)
        fl    = self.zone(scan,  25,  65)
        fr    = self.zone(scan, -65, -25)
        left  = self.zone(scan,  65, 120)
        right = self.zone(scan, -120, -65)

        thr = self.obs_thr
        emg = self.emg_thr

        # ============================================================
        # PRIORITY 1: Emergency collision — reverse immediately
        # Fires in ANY state except REVERSING
        # ============================================================
        if min(front, fl, fr) < emg and self.state != St.REVERSING:
            # Turn AWAY from the closest obstacle
            if fl <= fr:
                self.turn_dir = -1.0
            else:
                self.turn_dir = 1.0
            self.state      = St.REVERSING
            self._rev_timer = 0.8
            self.get_logger().warn(
                f'EMERGENCY  f={front:.2f} fl={fl:.2f} fr={fr:.2f}')

        # ============================================================
        # PRIORITY 2: Stuck detection — fires in ANY movement state
        # ============================================================
        elif self._stk_ticks >= self.stuck_lim \
                and self.state in (St.EXPLORING, St.AVOIDING):
            self.turn_dir   = 1.0 if right <= left else -1.0
            self.state      = St.RECOVERING
            self._stk_ticks = 0
            self.get_logger().warn('Stuck — recovery spin')

        # ============================================================
        # STATE MACHINE
        # ============================================================

        if self.state == St.EXPLORING:
            # Check for obstacles in the forward hemisphere
            obstacle_ahead = (front < thr
                              or fl < thr * 0.75
                              or fr < thr * 0.75)

            if obstacle_ahead:
                # IMMEDIATELY stop and turn
                self.turn_dir     = self._pick_clear_direction(scan)
                self.state        = St.AVOIDING
                self._avoid_timer = 3.0    # Max 3s, exits early when clear
                side = 'L' if self.turn_dir > 0 else 'R'
                self.get_logger().info(
                    f'Obstacle  f={front:.2f} fl={fl:.2f} fr={fr:.2f}'
                    f' → turn {side}')
            else:
                # Drive forward with wall-following
                twist.linear.x = self.v_lin

                # Slow down when obstacles are moderately close
                closest_fwd = min(front, fl, fr)
                if closest_fwd < thr * 1.5:
                    twist.linear.x *= 0.5

                # Wall-following correction
                if left < self.side_thr:
                    self._wall_hz = -0.40
                elif right < self.side_thr:
                    self._wall_hz = +0.40
                else:
                    self._wall_hz *= 0.80
                    if abs(self._wall_hz) < 0.02:
                        self._wall_hz = 0.0

                twist.angular.z = self._wall_hz + random.uniform(-0.03, 0.03)

        elif self.state == St.AVOIDING:
            self._avoid_timer -= self.DT
            twist.angular.z = self.v_ang * self.turn_dir

            # Re-check every tick: is the forward path now clear?
            clear = (front > thr * 1.3
                     and fl  > thr * 0.9
                     and fr  > thr * 0.9)

            if clear and self._avoid_timer < 2.7:
                # Path is clear AND we've turned for at least 0.3s
                self.state = St.EXPLORING
            elif self._avoid_timer <= 0:
                # Max avoidance time reached, resume anyway
                self.get_logger().warn('Avoidance timeout — resuming')
                self.state = St.EXPLORING

        elif self.state == St.REVERSING:
            self._rev_timer -= self.DT
            twist.linear.x  = -self.v_lin * 0.7
            twist.angular.z = self.v_ang * 0.5 * self.turn_dir

            if self._rev_timer <= 0:
                # After reversing, turn to find clear direction
                self.turn_dir     = self._pick_clear_direction(scan)
                self.state        = St.AVOIDING
                self._avoid_timer = 2.5

        elif self.state == St.RECOVERING:
            twist.angular.z = self.v_ang * self.turn_dir
            # Exit when forward is clear
            if front > thr * 1.2 and fl > thr * 0.9 and fr > thr * 0.9:
                self.state = St.EXPLORING
                self._stk_ticks = 0

        self.cmd_pub.publish(twist)

        # Status
        s = String()
        s.data = (f'{self.state}  cells={self.grid.count()}/{self.cov_stop}'
                  f'  f={front:.2f} fl={fl:.2f} fr={fr:.2f}'
                  f'  l={left:.2f} r={right:.2f}')
        self.status_pub.publish(s)


# Entry point

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
