#!/usr/bin/env python3
"""
Simple rover movement test script
Publishes various movement commands to test the 6-wheeled rover with ackermann steering
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class RoverMovementTest(Node):
    def __init__(self):
        super().__init__('rover_movement_test')
        
        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Test sequence variables
        self.start_time = time.time()
        self.test_phase = 0
        self.phase_duration = 3.0  # seconds per phase
        
        self.get_logger().info('Rover Movement Test Node Started')
        self.get_logger().info('Test sequence:')
        self.get_logger().info('  Phase 0 (0-3s): Forward movement')
        self.get_logger().info('  Phase 1 (3-6s): Backward movement') 
        self.get_logger().info('  Phase 2 (6-9s): Turn left (in place)')
        self.get_logger().info('  Phase 3 (9-12s): Turn right (in place)')
        self.get_logger().info('  Phase 4 (12-15s): Forward with left turn')
        self.get_logger().info('  Phase 5 (15-18s): Forward with right turn')
        self.get_logger().info('  Phase 6 (18s+): Stop')

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Create twist message
        cmd = Twist()
        
        # Determine current test phase
        phase = int(elapsed_time // self.phase_duration)
        
        if phase != self.test_phase:
            self.test_phase = phase
            self.get_logger().info(f'Starting phase {phase}')
        
        # Define movement for each phase
        if phase == 0:
            # Forward movement
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            
        elif phase == 1:
            # Backward movement
            cmd.linear.x = -0.5
            cmd.angular.z = 0.0
            
        elif phase == 2:
            # Turn left (in place)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            
        elif phase == 3:
            # Turn right (in place)
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
            
        elif phase == 4:
            # Forward with left turn
            cmd.linear.x = 0.3
            cmd.angular.z = 0.3
            
        elif phase == 5:
            # Forward with right turn
            cmd.linear.x = 0.3
            cmd.angular.z = -0.3
            
        else:
            # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if phase == 6:
                self.get_logger().info('Test complete - rover stopped')
        
        # Publish command
        self.cmd_vel_publisher.publish(cmd)
        
        # Log current command every 1 second
        if int(elapsed_time) != int(elapsed_time - 0.1):
            self.get_logger().info(f'Phase {phase}: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    rover_test = RoverMovementTest()
    
    try:
        rclpy.spin(rover_test)
    except KeyboardInterrupt:
        rover_test.get_logger().info('Test interrupted by user')
    finally:
        # Send stop command
        stop_cmd = Twist()
        rover_test.cmd_vel_publisher.publish(stop_cmd)
        rover_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()