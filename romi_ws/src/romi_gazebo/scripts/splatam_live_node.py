import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import torch
import numpy as np

# Import SplatAM internals (from the SplatAM repo)
# from splatam.slam import SplatAM
# from configs.romi import config  # Your specific camera intrinsics

class SplatAMLiveNode(Node):
    def __init__(self):
        super().__init__('splatam_live_node')
        self.bridge = CvBridge()
        
        # Initialize SplatAM system here using your configuration
        # self.splatam = SplatAM(config)
        
        # Subscribe to the live topics from the laptop
        self.rgb_sub = message_filters.Subscriber(self, Image, '/depth_camera/image')
        self.depth_sub = message_filters.Subscriber(self, Image, '/depth_camera/depth_image')
        
        # Synchronize RGB and Depth frames
        # queue_size=2 ensures we drop old frames if the GPU mapping step takes too long
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=2, slop=0.05)
        self.ts.registerCallback(self.live_frame_callback)
        
        self.get_logger().info("SplatAM Live Bridge active. Waiting for Romi sensor data...")

    def live_frame_callback(self, rgb_msg, depth_msg):
        # 1. Convert ROS messages to OpenCV matrices
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        
        # The live Gazebo depth topic is 32FC1 (meters), not 16UC1 (mm)
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 
        
        # 2. Convert to GPU PyTorch Tensors for SplatAM
        # SplatAM expects RGB as (C, H, W) normalized to [0, 1]
        rgb_tensor = torch.from_numpy(cv_rgb).permute(2, 0, 1).float() / 255.0
        depth_tensor = torch.from_numpy(cv_depth).float() # Already in meters
        
        # In actual SplatAM environment, uncomment these lines:
        # rgb_tensor = rgb_tensor.cuda()
        # depth_tensor = depth_tensor.cuda()
        
        # === SPLATAM ONLINE EXECUTION ===
        
        # Step A: Tracking (Estimate the new camera pose)
        # pose = self.splatam.track(rgb_tensor, depth_tensor)
        
        # Step B: Mapping (Densify Gaussians and optimize the 3D map)
        # self.splatam.map(rgb_tensor, depth_tensor, pose)
        
        self.get_logger().info("Processed live frame & updated 3D Gaussians.")

def main(args=None):
    rclpy.init(args=args)
    node = SplatAMLiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
