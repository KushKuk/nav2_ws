#!/usr/bin/env python3
"""
Rover Control and Monitoring Script
This script monitors the rover's status and allows manual control
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.drive_commands_sub = self.create_subscription(
            Float64MultiArray, '/drive_controller/commands', self.drive_commands_callback, 10)
        self.steer_commands_sub = self.create_subscription(
            Float64MultiArray, '/steer_controller/commands', self.steer_commands_callback, 10)
        
        # Status variables
        self.joint_states = None
        self.drive_commands = None
        self.steer_commands = None
        self.last_status_time = time.time()
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('Rover Controller Started')
        self.get_logger().info('Monitoring rover status...')
        
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
        self.get_logger().info("=== ROVER STATUS ===")
        
        if self.joint_states:
            self.get_logger().info(f"Joint States: {len(self.joint_states.name)} joints detected")
            # Print wheel speeds (last 6 velocities should be wheels)
            if len(self.joint_states.velocity) >= 6:
                wheel_vels = self.joint_states.velocity[-6:]
                self.get_logger().info(f"Wheel Velocities: {[f'{v:.2f}' for v in wheel_vels]}")
        
        if self.drive_commands:
            self.get_logger().info(f"Drive Commands: {[f'{c:.2f}' for c in self.drive_commands.data]}")
            
        if self.steer_commands:
            self.get_logger().info(f"Steer Commands: {[f'{c:.2f}' for c in self.steer_commands.data]}")
    
    def send_movement_command(self, linear_x=0.0, angular_z=0.0):
        """Send movement command to rover"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f"Sent command: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    
    rover_controller = RoverController()
    
    # Initial test sequence
    test_commands = [
        (0.5, 0.0, "Forward"),
        (0.0, 0.5, "Turn Left"),
        (0.0, -0.5, "Turn Right"),
        (0.3, 0.3, "Forward + Left Turn"),
        (0.0, 0.0, "Stop")
    ]
    
    rover_controller.get_logger().info("Starting automatic test sequence...")
    
    try:
        # Run test sequence
        for i, (linear_x, angular_z, description) in enumerate(test_commands):
            rover_controller.get_logger().info(f"Test {i+1}/5: {description}")
            rover_controller.send_movement_command(linear_x, angular_z)
            
            # Spin for 3 seconds while monitoring
            start_time = time.time()
            while time.time() - start_time < 3.0:
                rclpy.spin_once(rover_controller, timeout_sec=0.1)
        
        rover_controller.get_logger().info("Test sequence complete. Monitoring continues...")
        rover_controller.get_logger().info("Use 'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...' to send manual commands")
        
        # Continue monitoring
        rclpy.spin(rover_controller)
        
    except KeyboardInterrupt:
        rover_controller.get_logger().info('Controller stopped by user')
    finally:
        # Send stop command
        rover_controller.send_movement_command(0.0, 0.0)
        rover_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()