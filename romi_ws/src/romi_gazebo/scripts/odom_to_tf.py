#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.declare_parameter('odom_topic', '/model/romi/odometry')
        self.declare_parameter('parent_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.subscription = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry) -> None:
        transform = TransformStamped()
        # Use the node's clock (sim time when use_sim_time=true) to stay
        # in sync with robot_state_publisher and other TF publishers.
        # Using msg.header.stamp causes TF_OLD_DATA because the Gazebo
        # bridge delivers odom messages with timestamps that lag behind
        # the /clock topic.
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = msg.header.frame_id or self.parent_frame_id
        transform.child_frame_id = msg.child_frame_id or self.child_frame_id

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        transform.transform.rotation = msg.pose.pose.orientation

        self.broadcaster.sendTransform(transform)


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