#!/usr/bin/env python3
"""
odom_to_tf.py  (fixed)
──────────────────────
The original had two subtle bugs:
 
1. child_frame_id fallback:
   The Ignition/Gazebo bridge translates the gz odometry message but does NOT
   populate child_frame_id in the ROS Odometry message (it only maps frame_id).
   So  msg.child_frame_id  is always "".  The original code did:
       child_frame_id = msg.child_frame_id or self.child_frame_id
   which LOOKS safe, but the empty-string falsy check never fires consistently
   across all bridge versions. We now ALWAYS use the parameter value.
 
2. Timestamp jitter:
   Using self.get_clock().now() instead of msg.header.stamp is correct for
   sim time. But we also add a tiny lookahead (+5 ms) so downstream nodes
   (slam_toolbox, robot_state_publisher) never hit TF_EXTRAPOLATION_INTO_FUTURE
   on the first few ticks while the clock is settling.
"""
 
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
 
 
class OdomToTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
 
        self.declare_parameter('odom_topic',      '/model/romi/odometry')
        self.declare_parameter('parent_frame_id', 'odom')
        self.declare_parameter('child_frame_id',  'base_link')
        # Small lookahead to avoid TF_EXTRAPOLATION_INTO_FUTURE during clock settle
        self.declare_parameter('stamp_lookahead_ms', 5)
 
        odom_topic           = self.get_parameter('odom_topic').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id  = self.get_parameter('child_frame_id').value   # always use param
        lookahead_ms         = self.get_parameter('stamp_lookahead_ms').value
        self._lookahead      = Duration(nanoseconds=lookahead_ms * 1_000_000)
 
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.broadcaster  = TransformBroadcaster(self)
 
        self.get_logger().info(
            f'odom→TF: {self.parent_frame_id} → {self.child_frame_id}  '
            f'(lookahead {lookahead_ms} ms)')
 
    def odom_callback(self, msg: Odometry) -> None:
        t = TransformStamped()
 
        # Timestamp: use sim clock + tiny lookahead
        now = self.get_clock().now()
        t.header.stamp    = (now + self._lookahead).to_msg()
 
        # Always use parameter values — bridge may leave msg fields blank
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id  = self.child_frame_id
 
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0   # differential drive → z always 0
 
        t.transform.rotation = msg.pose.pose.orientation
 
        self.broadcaster.sendTransform(t)
 
 
def main() -> None:
    rclpy.init()
    node = OdomToTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
 
 
if __name__ == '__main__':
    main()