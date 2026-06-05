#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.subscription = self.create_subscription(
            Image, '/depth_camera/image', self.listener_callback, 10)
        self.bridge = CvBridge()
        
        # Setup OpenCV Video Writer (Saves as output_video.avi)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('output_video.avi', fourcc, 15.0, (640, 480))
        self.get_logger().info('Recording video to output_video.avi...')

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.out.write(cv_image)
        
        # Display the live feed
        cv2.imshow("Robot Camera View", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Clean up and save the video file
    node.get_logger().info('Saving video file...')
    node.out.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
