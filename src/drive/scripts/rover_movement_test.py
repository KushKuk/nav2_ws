#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time

class RoverTester(Node):
    def __init__(self):
        super().__init__('rover_tester')
        
        # Publishers for direct control
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("=== ROVER MOVEMENT TESTER ===")
        self.get_logger().info("This script will test rover movement step by step")
        
        # Wait for publishers to be ready
        time.sleep(2)
        
        self.run_tests()
        
    def run_tests(self):
        self.get_logger().info("\n--- TEST 1: Direct Drive Controller Commands ---")
        self.test_direct_drive_commands()
        
        time.sleep(3)
        
        self.get_logger().info("\n--- TEST 2: CMD_VEL Commands ---")
        self.test_cmd_vel_commands()
        
        time.sleep(3)
        
        self.get_logger().info("\n--- TEST 3: Continuous Movement ---")
        self.test_continuous_movement()
        
    def test_direct_drive_commands(self):
        """Test direct drive controller commands"""
        
        # Test 1: All wheels forward
        self.get_logger().info("Sending all wheels forward (speed: 1.0 rad/s)")
        drive_msg = Float64MultiArray()
        drive_msg.data = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # All wheels forward
        
        for i in range(10):  # Send for 2 seconds
            self.drive_pub.publish(drive_msg)
            time.sleep(0.2)
        
        # Stop
        self.get_logger().info("Stopping all wheels")
        drive_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(5):
            self.drive_pub.publish(drive_msg)
            time.sleep(0.2)
            
        time.sleep(1)
        
        # Test 2: Left wheels only
        self.get_logger().info("Sending left wheels only (speed: 1.0 rad/s)")
        drive_msg.data = [1.0, 0.0, 1.0, 0.0, 1.0, 0.0]  # Left wheels only
        
        for i in range(10):
            self.drive_pub.publish(drive_msg)
            time.sleep(0.2)
            
        # Stop
        drive_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(5):
            self.drive_pub.publish(drive_msg)
            time.sleep(0.2)
    
    def test_cmd_vel_commands(self):
        """Test cmd_vel commands"""
        
        # Test 1: Forward motion
        self.get_logger().info("Sending forward cmd_vel (linear.x: 0.5 m/s)")
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.5
        cmd_msg.angular.z = 0.0
        
        for i in range(10):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
        
        # Stop
        self.get_logger().info("Stopping cmd_vel")
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        for i in range(5):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
            
        time.sleep(1)
        
        # Test 2: Rotation
        self.get_logger().info("Sending rotation cmd_vel (angular.z: 0.5 rad/s)")
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.5
        
        for i in range(10):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
        
        # Stop
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        for i in range(5):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
    
    def test_continuous_movement(self):
        """Test continuous movement pattern"""
        
        self.get_logger().info("Starting continuous movement test...")
        self.get_logger().info("Pattern: Forward -> Turn Left -> Forward -> Turn Right -> Stop")
        
        cmd_msg = Twist()
        
        # Forward
        self.get_logger().info("Moving forward...")
        cmd_msg.linear.x = 0.3
        cmd_msg.angular.z = 0.0
        for i in range(15):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
        
        # Turn left
        self.get_logger().info("Turning left...")
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.3
        for i in range(10):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
        
        # Forward again
        self.get_logger().info("Moving forward again...")
        cmd_msg.linear.x = 0.3
        cmd_msg.angular.z = 0.0
        for i in range(15):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
        
        # Turn right
        self.get_logger().info("Turning right...")
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = -0.3
        for i in range(10):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
        
        # Stop
        self.get_logger().info("Final stop...")
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        for i in range(10):
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(0.2)
        
        self.get_logger().info("=== MOVEMENT TEST COMPLETED ===")

def main(args=None):
    rclpy.init(args=args)
    tester = RoverTester()
    
    # Keep node alive briefly to complete tests
    time.sleep(1)
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()