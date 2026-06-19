import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from cv_bridge import CvBridge
import numpy as np
import torch
import threading
import time

# Import the core SplaTAM SLAM engine
from splatam.slam import SplatAM
from configs.romi_live import config

class ROS2BackgroundStreamer(Node):
    """ Runs entirely in the background, keeping the freshest frame ready """
    def __init__(self):
        super().__init__('splatam_ros2_streamer')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        self.rgb_sub = message_filters.Subscriber(self, Image, '/depth_camera/image')
        self.depth_sub = message_filters.Subscriber(self, Image, '/depth_camera/depth_image')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=2, slop=0.05)
        self.ts.registerCallback(self.sensor_cb)

    def sensor_cb(self, rgb_msg, depth_msg):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") 
            
            with self.frame_lock:
                self.latest_frame = (cv_rgb, cv_depth)
        except Exception as e:
            self.get_logger().error(f"Frame conversion failed: {e}")

def run_ros_node(node):
    rclpy.spin(node)

def main():
    # 1. Start ROS 2 Network Streamer in the Background Thread
    rclpy.init()
    streamer_node = ROS2BackgroundStreamer()
    ros_thread = threading.Thread(target=run_ros_node, args=(streamer_node,))
    ros_thread.daemon = True
    ros_thread.start()
    
    print("[INFO] ROS 2 Streamer connected. Booting SplaTAM GUI...")
    
    # 2. Initialize SplaTAM on the Main Thread (Required for GUI to open)
    splatam = SplatAM(config)
    
    frame_idx = 0
    while True:
        frame = None
        
        # Safely grab the newest network frame
        with streamer_node.frame_lock:
            if streamer_node.latest_frame is not None:
                frame = streamer_node.latest_frame
                streamer_node.latest_frame = None 
        
        if frame is None:
            time.sleep(0.01)
            continue
            
        cv_rgb, cv_depth = frame
        
        # Convert to GPU Tensors [0, 1]
        rgb_tensor = torch.from_numpy(cv_rgb).permute(2, 0, 1).float().cuda() / 255.0
        depth_tensor = torch.from_numpy(cv_depth).float().cuda()
        
        if frame_idx == 0:
            print("[INFO] First frame received. Initializing 3D Gaussian Map...")
            splatam.initialize_map(rgb_tensor, depth_tensor)
        else:
            print(f"[INFO] Tracking & Mapping Frame {frame_idx}...")
            # Optimize camera pose
            current_pose = splatam.track(rgb_tensor, depth_tensor)
            # Densify Gaussians
            splatam.map(rgb_tensor, depth_tensor, current_pose)
            
        # 3. Trigger the Live Visualizer Update
        # This pushes the newly optimized 3D Gaussians to the graphics card for rendering
        if config.get("viz", False):
            if hasattr(splatam, 'update_viewer'):
                splatam.update_viewer(current_pose)
            elif hasattr(splatam, 'render_gui'):
                splatam.render_gui()
                
        frame_idx += 1

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down SplaTAM live visualizer...")
        rclpy.shutdown()
