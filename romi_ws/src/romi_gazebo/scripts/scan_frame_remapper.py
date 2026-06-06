#!/usr/bin/env python3
"""
scan_frame_remapper.py
─────────────────────
Gazebo Fortress publishes LaserScan with frame_id set to the full scoped
sensor name:  "romi::sensor_mount::lidar"
 
slam_toolbox looks up this frame in the TF tree but our static TF
publishers use the ROS-style slash path:  "romi/sensor_mount/lidar"
 
The two strings never match → SLAM silently drops every scan or
misaligns all scans to the map origin, creating ghost walls and
the rotated/split map you see.
 
This node:
  • Subscribes to  /lidar/scan          (raw from bridge)
  • Republishes to /lidar/scan_fixed    (corrected frame_id)
 
Add to launch, then point slam_toolbox at /lidar/scan_fixed.
"""
 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
 
 
GAZEBO_FRAME = 'romi::sensor_mount::lidar'   # what Gazebo publishes
ROS_FRAME    = 'romi/sensor_mount/lidar'     # what our TF tree has
 
 
class ScanFrameRemapper(Node):
    def __init__(self):
        super().__init__('scan_frame_remapper')
 
        self.declare_parameter('input_topic',    '/lidar/scan')
        self.declare_parameter('output_topic',   '/lidar/scan_fixed')
        self.declare_parameter('target_frame_id', ROS_FRAME)
 
        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame_id').value
 
        # Use best-effort on input to match Gazebo bridge QoS
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
 
        self.sub = self.create_subscription(
            LaserScan, in_topic, self.scan_cb, sensor_qos)
        self.pub = self.create_publisher(LaserScan, out_topic, 10)
 
        self._warned = False
        self.get_logger().info(
            f'scan_frame_remapper: {in_topic} → {out_topic} '
            f'(frame_id → "{self.target_frame}")')
 
    def scan_cb(self, msg: LaserScan):
        if not self._warned and msg.header.frame_id != self.target_frame:
            self.get_logger().info(
                f'Remapping frame_id: "{msg.header.frame_id}" → "{self.target_frame}"')
            self._warned = True
 
        # Mutate a shallow copy — avoids allocating a new message every scan
        msg.header.frame_id = self.target_frame
        self.pub.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRemapper()
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
