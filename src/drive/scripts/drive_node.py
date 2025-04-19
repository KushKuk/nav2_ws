#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
from select import select

speed = 1.0

class CrabSteeringTeleop(Node):
    def __init__(self):
        super().__init__('crab_steering_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_x = 0.0
        self.linear_y = 0.0

        self.get_logger().info("Crab Steering Teleop started. Use WASD to move, Space to stop.")

        # Start key listener thread
        self.running = True
        self.key_thread = threading.Thread(target=self.key_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

        # Send Twist periodically
        self.timer = self.create_timer(0.1, self.publish_twist)

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        self.pub.publish(msg)

    def key_listener(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                rlist, _, _ = select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    self.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def process_key(self, key):
        if key == 'w':
            self.linear_x = speed
            self.linear_y = 0.0
            self.get_logger().info("Forward")
        elif key == 's':
            self.linear_x = -speed
            self.linear_y = 0.0
            self.get_logger().info("Backward")
        elif key == 'a':
            self.linear_y = speed
            self.linear_x = 0.0
            self.get_logger().info("Left (Crab)")
        elif key == 'd':
            self.linear_y = -speed
            self.linear_x = 0.0
            self.get_logger().info("Right (Crab)")
        elif key == ' ':
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.get_logger().info("Stop")
        elif key == '\x03':  # CTRL+C
            self.running = False
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CrabSteeringTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.key_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
