#!/usr/bin/env python3
"""
Rover Status Monitor - No automatic movement
This script only monitors the rover's status without sending commands
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class RoverStatusMonitor(Node):
    def __init__(self):
        super().__init__('rover_status_monitor')
        
        # Subscribers only - no publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.drive_commands_sub = self.create_subscription(
            Float64MultiArray, '/drive_controller/commands', self.drive_commands_callback, 10)
        self.steer_commands_sub = self.create_subscription(
            Float64MultiArray, '/steer_controller/commands', self.steer_commands_callback, 10)
        
        # Status variables
        self.current_cmd_vel = None
        self.joint_states = None
        self.drive_commands = None
        self.steer_commands = None
        
        # Timer for status updates
        self.status_timer = self.create_timer(3.0, self.print_status)
        
        self.get_logger().info('Rover Status Monitor Started')
        self.get_logger().info('Monitoring rover without sending any commands...')
        
    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel commands"""
        self.current_cmd_vel = msg
        
    def joint_states_callback(self, msg):
        """Callback for joint states"""
        self.joint_states = msg
        
    def drive_commands_callback(self, msg):
        """Callback for drive controller commands"""
        self.drive_commands = msg
        
    def steer_commands_callback(self, msg):
        """Callback for steering controller commands"""
        self.steer_commands = msg
        
    def print_status(self):
        """Print current rover status"""
        self.get_logger().info("=== ROVER STATUS MONITOR ===")
        
        if self.current_cmd_vel:
            self.get_logger().info(f"Current cmd_vel: linear.x={self.current_cmd_vel.linear.x:.3f}, angular.z={self.current_cmd_vel.angular.z:.3f}")
        else:
            self.get_logger().info("No cmd_vel commands received yet")
            
        if self.drive_commands:
            drive_summary = [f'{c:.2f}' for c in self.drive_commands.data[:3]]  # Show first 3 wheels
            self.get_logger().info(f"Wheel speeds (first 3): {drive_summary}")
            
        if self.steer_commands:
            steer_summary = [f'{c:.2f}' for c in self.steer_commands.data[:3]]  # Show first 3 steering angles
            self.get_logger().info(f"Steering angles (first 3): {steer_summary}")
            
        self.get_logger().info("Differential steering system: ACTIVE")
        self.get_logger().info("---")


def main(args=None):
    rclpy.init(args=args)
    
    monitor = RoverStatusMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Status monitor stopped by user')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()