#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Save terminal settings
settings = termios.tcgetattr(sys.stdin)

def getKey(timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    # Wait for input for 'timeout' seconds
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b': # Handle arrow key escape sequences
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class HoldToMoveTeleop(Node):
    def __init__(self):
        super().__init__('hold_to_move_teleop')
        self.pub = self.create_publisher(Twist, '/model/romi/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Ready! HOLD Arrow Keys to move. RELEASE to stop. (CTRL-C to quit)")

    def loop(self):
        key = getKey(0.1)
        twist = Twist()
        
        if key == '\x1b[A':   # Up Arrow
            twist.linear.x = 0.5
        elif key == '\x1b[B': # Down Arrow
            twist.linear.x = -0.5
        elif key == '\x1b[D': # Left Arrow
            twist.angular.z = 1.0
        elif key == '\x1b[C': # Right Arrow
            twist.angular.z = -1.0
        elif key == '\x03':   # CTRL-C
            rclpy.shutdown()
            return
            
        # Publishes the movement, or 0.0 if no key is held
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HoldToMoveTeleop()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
