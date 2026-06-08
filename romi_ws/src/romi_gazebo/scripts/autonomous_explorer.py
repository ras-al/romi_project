#!/usr/bin/env python3
"""
Autonomous reactive explorer for the Romi robot.

Uses reactive potential-field steering with a gap-width check.
The robot continuously curves away from nearby obstacles AND refuses
to enter passages narrower than its body.

Robot dimensions (from SDF):
  Chassis collision box: 0.16 × 0.12 × 0.04 m
  Wheel-to-wheel width: ~0.13 m
  LiDAR at center (0, 0, 0.05)
  → Robot body extends 0.08 m from LiDAR center to side edge
  → Minimum safe passage: 0.13 m body + 0.12 m margin = 0.25 m each side

States:
  WAITING   — no scan data yet
  DRIVING   — moving forward with proportional obstacle-reactive steering
  REVERSING — backing away from very close obstacle (< emg_thr)
  DONE      — exploration complete
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


class CoverageGrid:
    def __init__(self, cell_size: float = 0.5):
        self.cell = cell_size
        self.visited: set = set()

    def mark(self, x: float, y: float):
        self.visited.add((int(x / self.cell), int(y / self.cell)))

    def count(self) -> int:
        return len(self.visited)

    def score_direction(self, x: float, y: float, yaw: float,
                        offset: float, lookahead: float = 5.0) -> int:
        angle = yaw + offset
        steps = int(lookahead / self.cell)
        score = 0
        for s in range(1, steps + 1):
            cx = int((x + s * self.cell * math.cos(angle)) / self.cell)
            cy = int((y + s * self.cell * math.sin(angle)) / self.cell)
            if (cx, cy) not in self.visited:
                score += max(1, steps - s + 1)
        return score


class St:
    WAITING   = 'WAITING'
    DRIVING   = 'DRIVING'
    REVERSING = 'REVERSING'
    DONE      = 'DONE'


# ── Robot physical constants ──────────────────────────────────────
ROBOT_HALF_WIDTH = 0.08    # LiDAR center to side edge (from SDF)
BODY_CLEARANCE   = 0.12    # extra margin on each side
MIN_SIDE_DIST    = ROBOT_HALF_WIDTH + BODY_CLEARANCE   # 0.20 m


class ReactiveExplorer(Node):

    def __init__(self):
        super().__init__('reactive_explorer')

        self.declare_parameter('cmd_vel_topic',       '/model/romi/cmd_vel')
        self.declare_parameter('scan_topic',          '/lidar/scan')
        self.declare_parameter('odom_topic',          '/model/romi/odometry')
        self.declare_parameter('linear_speed',         0.18)
        self.declare_parameter('angular_speed',        1.5)
        self.declare_parameter('obstacle_threshold',   0.50)
        self.declare_parameter('emergency_threshold',  0.20)
        self.declare_parameter('side_threshold',       0.35)
        self.declare_parameter('coverage_stop_cells',  600)
        self.declare_parameter('exploration_timeout',  0.0)
        self.declare_parameter('coverage_cell_size',   0.5)
        self.declare_parameter('progress_watchdog_s',  60.0)

        self.v_lin     = self.get_parameter('linear_speed').value
        self.v_ang     = self.get_parameter('angular_speed').value
        self.obs_thr   = self.get_parameter('obstacle_threshold').value
        self.emg_thr   = self.get_parameter('emergency_threshold').value
        self.side_thr  = self.get_parameter('side_threshold').value
        self.cov_stop  = self.get_parameter('coverage_stop_cells').value
        self.exp_to    = self.get_parameter('exploration_timeout').value
        self.prog_wd   = self.get_parameter('progress_watchdog_s').value

        cmd = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub    = self.create_publisher(Twist, cmd, 10)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)

        self.state       = St.WAITING
        self.latest_scan = None
        self.rec_on      = False
        self.DT          = 0.05
        self._rev_timer  = 0.0
        self._rev_dir    = 1.0
        self.rx = self.ry = self.ryaw = 0.0
        self._close_timer = 0.0
        self.grid = CoverageGrid(self.get_parameter('coverage_cell_size').value)
        self._last_cov_count = 0
        self._last_cov_time  = _time.monotonic()
        self._t0 = None

        self.create_timer(self.DT, self.loop)
        self.get_logger().info(
            f'Explorer  obs={self.obs_thr:.2f}  emg={self.emg_thr:.2f}  '
            f'side_min={MIN_SIDE_DIST:.2f}  body={ROBOT_HALF_WIDTH:.2f}')

    # ── Scan helpers ──────────────────────────────────────────────

    def _zone(self, scan, d0, d1):
        """Min valid range in [d0°, d1°]. 0° = forward."""
        inc, amin = scan.angle_increment, scan.angle_min
        i0 = max(0, int((math.radians(d0) - amin) / inc))
        i1 = min(len(scan.ranges) - 1, int((math.radians(d1) - amin) / inc))
        if i0 > i1: i0, i1 = i1, i0
        v = [r for r in scan.ranges[i0:i1+1]
             if math.isfinite(r) and scan.range_min < r < scan.range_max]
        return min(v) if v else float('inf')

    def _closest_in_front_half(self, scan):
        """Min range in the forward 180° hemisphere."""
        return self._zone(scan, -90, 90)

    # ── Recording ─────────────────────────────────────────────────

    def _toggle_rec(self, on):
        if on == self.rec_on:
            return
        cli = self.create_client(SetBool, 'toggle_recording')
        if not cli.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request(); req.data = on
        cli.call_async(req)
        self.rec_on = on

    def _finish(self):
        self.get_logger().info(
            f'Done — {self.grid.count()} cells. Saving map...')
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

    # ── Callbacks ─────────────────────────────────────────────────

    def scan_cb(self, msg):
        if self.state == St.WAITING:
            self.get_logger().info('LiDAR → DRIVING')
            self.state = St.DRIVING
            self._toggle_rec(True)
        self.latest_scan = msg

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rx, self.ry = p.x, p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        self.ryaw = math.atan2(siny, cosy)
        self.grid.mark(self.rx, self.ry)

    # ── Main loop (20 Hz) ─────────────────────────────────────────

    def loop(self):
        if self.state in (St.WAITING, St.DONE) or self.latest_scan is None:
            return

        if self._t0 is None:
            self._t0 = _time.monotonic()
        now = _time.monotonic()
        if self.exp_to > 0 and (now - self._t0) > self.exp_to:
            self._finish(); return
        if self.grid.count() >= self.cov_stop:
            self._finish(); return

        # Progress watchdog
        cc = self.grid.count()
        if cc > self._last_cov_count:
            self._last_cov_count = cc
            self._last_cov_time = now
        elif (now - self._last_cov_time) > self.prog_wd \
                and self.state == St.DRIVING:
            self._rev_dir = random.choice([1.0, -1.0])
            self.state = St.REVERSING
            self._rev_timer = 0.5
            self._last_cov_time = now

        scan  = self.latest_scan
        twist = Twist()

        # ── Read scan zones ───────────────────────────────────────
        front      = self._zone(scan, -20,  20)    # narrow front cone
        front_wide = self._zone(scan, -40,  40)    # wider front for early detect
        fl         = self._zone(scan,  20,  60)
        fr         = self._zone(scan, -60, -20)
        left       = self._zone(scan,  60, 120)
        right      = self._zone(scan, -120, -60)
        rear       = self._zone(scan, 150, 210)    # behind the robot

        thr = self.obs_thr
        emg = self.emg_thr
        closest_fwd = min(front, fl, fr)

        # ── Gap width check ───────────────────────────────────────
        # The passage ahead is only as wide as left + right distances.
        # If it's narrower than the robot body + margin, DON'T enter.
        gap_too_narrow = (left < MIN_SIDE_DIST or right < MIN_SIDE_DIST)

        # ── Scan-stuck timer ──────────────────────────────────────
        # If anything is within emg distance for > 0.5s, force reverse
        anything_touching = min(front, fl, fr, left, right) < emg
        if anything_touching:
            self._close_timer += self.DT
        else:
            self._close_timer = 0.0

        # ══════════════════════════════════════════════════════════
        #  REVERSE TRIGGER — fires from ANY state except DONE
        # ══════════════════════════════════════════════════════════
        need_reverse = (
            closest_fwd < emg                      # front very close
            or (left < emg and right < emg)        # squeezed from both sides
            or self._close_timer >= 0.5            # stuck against something
        )

        if need_reverse and self.state == St.DRIVING:
            if min(fl, left) <= min(fr, right):
                self._rev_dir = -1.0
            else:
                self._rev_dir = 1.0
            self.state = St.REVERSING
            self._rev_timer = 1.0
            self._close_timer = 0.0
            self._toggle_rec(False)
            self.get_logger().warn(
                f'REVERSE  f={front:.2f} fl={fl:.2f} fr={fr:.2f} '
                f'l={left:.2f} r={right:.2f}')

        elif need_reverse and self.state == St.REVERSING \
                and self._rev_timer <= 0:
            # Still jammed — reverse other direction
            self._rev_dir *= -1.0
            self._rev_timer = 1.2
            self._close_timer = 0.0
            self.get_logger().warn('Still jammed — flip reverse direction')

        # ══════════════════════════════════════════════════════════
        #  STATE EXECUTION
        # ══════════════════════════════════════════════════════════

        if self.state == St.DRIVING:
            self._toggle_rec(True)

            # ── Speed control ─────────────────────────────────────
            if closest_fwd > thr * 2.0:
                speed = self.v_lin                    # full speed
            elif closest_fwd > thr:
                frac = (closest_fwd - thr) / thr
                speed = self.v_lin * (0.3 + 0.7 * frac)  # gradual slow
            elif closest_fwd > emg:
                speed = self.v_lin * 0.15             # crawl
            else:
                speed = 0.0                           # should be REVERSING

            # If gap too narrow, also slow down
            if gap_too_narrow:
                speed = min(speed, self.v_lin * 0.1)

            twist.linear.x = speed

            # ── Steering ──────────────────────────────────────────
            steer = 0.0

            # 1. Repulsion from front-left / front-right
            if fl < thr:
                steer -= (thr - fl) / thr * self.v_ang * 0.8  # push right
            if fr < thr:
                steer += (thr - fr) / thr * self.v_ang * 0.8  # push left

            # 2. Repulsion from sides (critical for narrow gaps!)
            if left < self.side_thr:
                force = (self.side_thr - left) / self.side_thr
                steer -= force * self.v_ang * 0.6              # push right
            if right < self.side_thr:
                force = (self.side_thr - right) / self.side_thr
                steer += force * self.v_ang * 0.6              # push left

            # 3. If front is blocked, pick best open direction
            if front < thr:
                if fl > fr:
                    steer += self.v_ang * 0.5
                elif fr > fl:
                    steer -= self.v_ang * 0.5
                else:
                    # Both blocked — pick by coverage
                    steer += self.v_ang * 0.5 * random.choice([1.0, -1.0])

            # 4. If gap too narrow, steer AWAY from narrower side
            if gap_too_narrow:
                if left < right:
                    steer -= self.v_ang * 0.7          # push right hard
                else:
                    steer += self.v_ang * 0.7          # push left hard

            # 5. Gentle exploration jitter
            steer += random.uniform(-0.03, 0.03)

            # Clamp
            twist.angular.z = max(-self.v_ang, min(self.v_ang, steer))

        elif self.state == St.REVERSING:
            self._toggle_rec(False)
            self._rev_timer -= self.DT

            if self._rev_timer > 0:
                # Phase 1: reverse
                twist.linear.x  = -self.v_lin * 0.6
                twist.angular.z = self.v_ang * 0.4 * self._rev_dir
            else:
                # Phase 2: spin in place to find clear heading
                twist.linear.x  = 0.0
                twist.angular.z = self.v_ang * self._rev_dir

            # Exit: front half is clear enough AND sides have room
            front_ok = (front > thr and fl > thr * 0.7 and fr > thr * 0.7)
            sides_ok = (left > MIN_SIDE_DIST and right > MIN_SIDE_DIST)

            if front_ok and sides_ok and self._rev_timer <= 0:
                self.state = St.DRIVING
                self.get_logger().info('Reversed → DRIVING')

            # Safety timeout: after 4s total, force resume
            if self._rev_timer < -3.0:
                self.state = St.DRIVING
                self.get_logger().warn('Reverse timeout → DRIVING')

        self.cmd_pub.publish(twist)

        s = String()
        s.data = (f'{self.state}  c={self.grid.count()}/{self.cov_stop}'
                  f'  f={front:.2f} fl={fl:.2f} fr={fr:.2f}'
                  f'  l={left:.2f} r={right:.2f}'
                  f'  gap={"NARROW" if gap_too_narrow else "OK"}')
        self.status_pub.publish(s)


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
