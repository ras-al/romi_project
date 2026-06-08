#!/usr/bin/env python3
"""
autonomous_explorer.py
Hybrid Potential Field + State Machine for Autonomous Navigation
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
    """
    Maintains a spatial grid of visited areas to guide exploration
    and determine coverage completion.
    """
    def __init__(self, cell_size: float = 0.5):
        self.cell    = cell_size
        self.visited: set = set()

    def mark(self, x: float, y: float):
        self.visited.add((int(x / self.cell), int(y / self.cell)))

    def count(self) -> int:
        return len(self.visited)

    def novelty_score(self, x: float, y: float,
                      yaw: float, offset: float,
                      lookahead: float = 4.0) -> float:
        """
        Calculates a novelty score [0, 1] for a candidate direction,
        weighting proximal unexplored cells higher.
        """
        angle = yaw + offset
        steps = max(1, int(lookahead / self.cell))
        score = 0.0
        max_score = 0.0
        for s in range(1, steps + 1):
            weight = steps - s + 1
            max_score += weight
            cx = int((x + s * self.cell * math.cos(angle)) / self.cell)
            cy = int((y + s * self.cell * math.sin(angle)) / self.cell)
            if (cx, cy) not in self.visited:
                score += weight
        return score / max_score if max_score > 0 else 0.0

class St:
    WAITING   = 'WAITING'
    DRIVING   = 'DRIVING'    # Potential field steering
    SPINNING  = 'SPINNING'   # Decisive in-place turn
    REVERSING = 'REVERSING'  # Collision recovery
    DONE      = 'DONE'

# Robot kinematics constraints
ROBOT_HALF_WIDTH = 0.09    # Half of chassis width plus margin
MIN_PASSAGE      = 0.30    # Minimum traversable lateral gap

class ReactiveExplorer(Node):

    def __init__(self):
        super().__init__('reactive_explorer')

        self.declare_parameter('cmd_vel_topic',       '/model/romi/cmd_vel')
        self.declare_parameter('scan_topic',          '/lidar/scan')
        self.declare_parameter('odom_topic',          '/odometry/filtered')
        self.declare_parameter('linear_speed',         0.20)
        self.declare_parameter('angular_speed',        1.4)
        self.declare_parameter('obstacle_threshold',   0.55)  
        self.declare_parameter('emergency_threshold',  0.22)  
        self.declare_parameter('side_threshold',       0.40)  
        self.declare_parameter('coverage_stop_cells',  600)
        self.declare_parameter('exploration_timeout',  0.0)
        self.declare_parameter('coverage_cell_size',   0.5)
        self.declare_parameter('novelty_weight',       0.45)
        self.declare_parameter('stuck_dist_threshold', 0.08)
        self.declare_parameter('stuck_window_s',       5.0)
        self.declare_parameter('progress_watchdog_s',  50.0)

        self.v_lin         = self.get_parameter('linear_speed').value
        self.v_ang         = self.get_parameter('angular_speed').value
        self.obs_thr       = self.get_parameter('obstacle_threshold').value
        self.emg_thr       = self.get_parameter('emergency_threshold').value
        self.side_thr      = self.get_parameter('side_threshold').value
        self.cov_stop      = self.get_parameter('coverage_stop_cells').value
        self.exp_to        = self.get_parameter('exploration_timeout').value
        self.novelty_w     = self.get_parameter('novelty_weight').value
        self.stuck_dist    = self.get_parameter('stuck_dist_threshold').value
        self.stuck_win     = self.get_parameter('stuck_window_s').value
        self.prog_wd       = self.get_parameter('progress_watchdog_s').value

        cmd = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub    = self.create_publisher(Twist,  cmd, 10)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)

        self.state       = St.WAITING
        self.scan        = None
        self.rec_on      = False
        self.DT          = 0.05           

        self.rx = self.ry = self.ryaw = 0.0

        self.spin_dir    = 1.0            
        self.spin_budget = 0.0            

        self.rev_dir     = 1.0
        self.rev_timer   = 0.0
        self.rev_phase   = 'back'         

        self._stuck_x    = None
        self._stuck_y    = None
        self._stuck_t    = _time.monotonic()

        self.grid        = CoverageGrid(
            self.get_parameter('coverage_cell_size').value)
        self._last_cov   = 0
        self._last_cov_t = _time.monotonic()

        self._t0         = None
        self._close_t    = 0.0

        self.create_timer(self.DT, self._loop)
        self.get_logger().info(
            f'Explorer ready  '
            f'obs={self.obs_thr:.2f}  emg={self.emg_thr:.2f}  '
            f'novelty_w={self.novelty_w:.2f}')

    def _zone(self, d0: float, d1: float) -> float:
        """Min valid LiDAR range in degree arc [d0, d1]. 0° = forward.
        Returns near-zero when obstacle is closer than LiDAR min_range."""
        scan = self.scan
        inc, amin = scan.angle_increment, scan.angle_min
        i0 = max(0, int((math.radians(d0) - amin) / inc))
        i1 = min(len(scan.ranges)-1, int((math.radians(d1) - amin) / inc))
        if i0 > i1: i0, i1 = i1, i0
        sector = scan.ranges[i0:i1+1]
        total = len(sector)
        if total == 0:
            return float('inf')

        valid = []
        below_min = 0
        for r in sector:
            if not math.isfinite(r):
                continue
            if r <= scan.range_min:
                below_min += 1
            elif r < scan.range_max:
                valid.append(r)

        # Even a single ray below min_range means something is at contact
        # distance in this sector (thin poles only block 1-3 rays)
        if below_min >= 1:
            return 0.05  # Contact distance

        return min(valid) if valid else float('inf')

    def _set_rec(self, on: bool):
        if on == self.rec_on:
            return
        cli = self.create_client(SetBool, 'toggle_recording')
        if not cli.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = on
        cli.call_async(req)
        self.rec_on = on

    def _finish(self):
        self.get_logger().info(
            f'Coverage goal reached ({self.grid.count()} cells). Stopping.')
        self.cmd_pub.publish(Twist())
        self._set_rec(False)
        import subprocess
        ts = _time.strftime('%Y%m%d_%H%M%S')
        subprocess.Popen([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', f'/tmp/romi_map_{ts}',
            '--ros-args', '-p', 'use_sim_time:=true'])
        self.state = St.DONE
        self.create_timer(
            3.5, lambda: (_ for _ in ()).throw(ExternalShutdownException()))

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg
        if self.state == St.WAITING:
            self.get_logger().info('LiDAR received — starting')
            self.state = St.DRIVING
            self._set_rec(True)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rx, self.ry = p.x, p.y
        sy = 2.0 * (q.w * q.z + q.x * q.y)
        cy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        self.ryaw = math.atan2(sy, cy)
        self.grid.mark(self.rx, self.ry)

    def _loop(self):
        if self.state in (St.WAITING, St.DONE) or self.scan is None:
            return

        if self._t0 is None:
            self._t0 = _time.monotonic()
        now = _time.monotonic()

        if self.exp_to > 0 and (now - self._t0) > self.exp_to:
            self._finish(); return
        if self.grid.count() >= self.cov_stop:
            self._finish(); return

        # Coverage Watchdog
        cc = self.grid.count()
        if cc > self._last_cov:
            self._last_cov   = cc
            self._last_cov_t = now
        elif (now - self._last_cov_t) > self.prog_wd \
                and self.state == St.DRIVING:
            self.get_logger().warn('Progress stalled — forcing SPINNING')
            self._start_spin(random.choice([1.0, -1.0]), duration=4.0)

        # Kinematic Displacement Watchdog
        if self._stuck_x is None:
            self._stuck_x, self._stuck_y = self.rx, self.ry
            self._stuck_t = now
        elif (now - self._stuck_t) > self.stuck_win:
            moved = math.hypot(self.rx - self._stuck_x, self.ry - self._stuck_y)
            self._stuck_x, self._stuck_y = self.rx, self.ry
            self._stuck_t = now
            if moved < self.stuck_dist and self.state == St.DRIVING:
                self.get_logger().warn(
                    f'Pose-stuck: moved only {moved:.3f}m in {self.stuck_win:.0f}s '
                    '— forcing SPINNING')
                left_near = self._zone(60, 120)
                right_near = self._zone(-120, -60)
                d = -1.0 if left_near < right_near else 1.0
                self._start_spin(d, duration=3.5)

        # Process Sensor Regions
        front = self._zone(-18,   18)
        fl    = self._zone( 18,   55)
        fr    = self._zone(-55,  -18)
        left  = self._zone( 55,  125)
        right = self._zone(-125, -55)
        rl    = self._zone( 125,  165)
        rr    = self._zone(-165, -125)

        thr = self.obs_thr
        emg = self.emg_thr

        passage_width = left + right
        passage_ok    = passage_width > MIN_PASSAGE

        # Global Collision Guard
        closest_fwd = min(front, fl, fr)

        if min(front, fl, fr, left, right) < emg:
            self._close_t += self.DT
        else:
            self._close_t = max(0.0, self._close_t - self.DT)

        needs_reverse = (
            closest_fwd < emg
            or (left < emg and right < emg)
            or self._close_t > 0.4             
        )

        if needs_reverse and self.state != St.REVERSING:
            d = -1.0 if min(fl, left) < min(fr, right) else 1.0
            self._start_reverse(d)

        twist = Twist()

        # State Execution
        if self.state == St.DRIVING:
            self._set_rec(True)

            # Dynamic Velocity Profiling
            if closest_fwd > thr * 2.0:
                speed = self.v_lin
            elif closest_fwd > thr:
                t = (closest_fwd - thr) / thr          
                speed = self.v_lin * (0.25 + 0.75 * t) 
            else:
                speed = self.v_lin * 0.12              

            if not passage_ok:
                speed = min(speed, self.v_lin * 0.08)

            twist.linear.x = speed

            # Potential Field Control
            steer = 0.0

            if fl < thr:
                force = (thr - fl) / thr
                steer -= force * self.v_ang * 0.9

            if fr < thr:
                force = (thr - fr) / thr
                steer += force * self.v_ang * 0.9

            if left < self.side_thr:
                force = (self.side_thr - left) / self.side_thr
                steer -= force * self.v_ang * 0.6

            if right < self.side_thr:
                force = (self.side_thr - right) / self.side_thr
                steer += force * self.v_ang * 0.6

            # Force Cancellation Prevention
            force_deadzone = self.v_ang * 0.15
            if front < thr and abs(steer) < force_deadzone:
                if left > right:
                    steer = self.v_ang * 0.8    
                else:
                    steer = -self.v_ang * 0.8   

            # Exploration Objective Vector
            novelty_l = self.grid.novelty_score(
                self.rx, self.ry, self.ryaw,  math.pi/4, lookahead=4.0)
            novelty_r = self.grid.novelty_score(
                self.rx, self.ry, self.ryaw, -math.pi/4, lookahead=4.0)

            novelty_bias = (novelty_l - novelty_r) * self.novelty_w * self.v_ang
            steer += novelty_bias

            steer += random.uniform(-0.04, 0.04)
            twist.angular.z = max(-self.v_ang, min(self.v_ang, steer))

            # Trigger Disentanglement
            if front < thr and speed < self.v_lin * 0.15:
                best_dir = 1.0 if left > right else -1.0
                self.get_logger().info(
                    f'Field insufficient (f={front:.2f}) → SPINNING')
                self._start_spin(best_dir, duration=3.0)

        elif self.state == St.SPINNING:
            self._set_rec(False)
            self.spin_budget -= self.DT

            twist.linear.x  = 0.0
            twist.angular.z = self.v_ang * self.spin_dir

            path_clear = (
                front > thr * 1.3
                and fl   > thr * 0.9
                and fr   > thr * 0.9
                and passage_ok
            )
            budget_exhausted = self.spin_budget <= 0

            if path_clear or budget_exhausted:
                if budget_exhausted and not path_clear:
                    self.get_logger().warn(
                        'Spin budget exhausted — resuming anyway')
                self.state = St.DRIVING

        elif self.state == St.REVERSING:
            self._set_rec(False)
            self.rev_timer -= self.DT

            if self.rev_phase == 'back' and self.rev_timer > 0:
                twist.linear.x  = -self.v_lin * 0.65
                twist.angular.z =  self.v_ang * 0.35 * self.rev_dir

            elif self.rev_phase == 'back' and self.rev_timer <= 0:
                self.rev_phase = 'spin'
                self.rev_timer = (3 * math.pi / 2) / self.v_ang

            elif self.rev_phase == 'spin':
                self.rev_timer -= self.DT
                twist.linear.x  = 0.0
                twist.angular.z = self.v_ang * self.rev_dir

                front_ok = front > thr and fl > thr * 0.8 and fr > thr * 0.8
                sides_ok = left > ROBOT_HALF_WIDTH and right > ROBOT_HALF_WIDTH
                time_ok  = self.rev_timer <= 0

                if (front_ok and sides_ok) or time_ok:
                    self.state = St.DRIVING
                    self._close_t = 0.0
                    self.get_logger().info('Recovery complete → DRIVING')

        self.cmd_pub.publish(twist)

        s = String()
        s.data = (
            f'{self.state}  cells={self.grid.count()}/{self.cov_stop}  '
            f'f={front:.2f} fl={fl:.2f} fr={fr:.2f}  '
            f'l={left:.2f} r={right:.2f}  '
            f'passage={"OK" if passage_ok else "NARROW"}'
        )
        self.status_pub.publish(s)

    def _start_spin(self, direction: float, duration: float):
        self.state       = St.SPINNING
        self.spin_dir    = direction
        self.spin_budget = duration         
        self._last_cov_t = _time.monotonic() 

    def _start_reverse(self, direction: float):
        self.state     = St.REVERSING
        self.rev_dir   = direction
        self.rev_timer = 1.2               
        self.rev_phase = 'back'
        self._close_t  = 0.0
        self.get_logger().warn(
            f'REVERSE  dir={"L" if direction>0 else "R"}')


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
