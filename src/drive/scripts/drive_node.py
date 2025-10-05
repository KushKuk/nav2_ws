#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
from select import select
import sys

speed = 1.0

class SkidSteeringTeleop(Node):
    def __init__(self):
        super().__init__('skid_steering_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_x = 0.0
        self.angular_z = 0.0

        print("\n[INFO] Skid Steering Teleop started. Use WASD to move, Space to stop.\n")

        self.running = True
        self.key_thread = threading.Thread(target=self.key_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

        self.timer = self.create_timer(0.1, self.publish_twist)

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
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

    def print_line(self, text):
        # Move cursor to start of line, clear line, then print
        sys.stdout.write('\r\033[K' + text + '\n')
        sys.stdout.flush()

    def process_key(self, key):
        if key == 'w':
            self.linear_x = speed
            self.angular_z = 0.0
            self.print_line("Forward")
        elif key == 's':
            self.linear_x = -speed
            self.angular_z = 0.0
            self.print_line("Backward")
        elif key == 'a':
            self.angular_z = -speed
            self.linear_x = 0.0
            self.print_line("Left")
        elif key == 'd':
            self.angular_z = speed
            self.linear_x = 0.0
            self.print_line("Right")
        elif key == ' ':
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.print_line("Stop")
        elif key == '\x03':  # CTRL+C
            self.running = False
            self.print_line("Exiting teleop...")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = SkidSteeringTeleop()
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
