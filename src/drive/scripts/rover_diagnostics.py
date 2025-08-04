#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
import time

class RoverDiagnostics(Node):
    def __init__(self):
        super().__init__('rover_diagnostics')
        
        # Subscribers to monitor topics
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.drive_cmd_sub = self.create_subscription(
            Float64MultiArray, '/drive_controller/commands', self.drive_cmd_callback, 10)
        
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Service client for controller manager
        self.controller_list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        
        # Status tracking
        self.cmd_vel_received = False
        self.drive_cmd_received = False
        self.joint_states_received = False
        self.last_cmd_vel = None
        self.last_drive_cmd = None
        self.last_joint_states = None
        
        # Timer for periodic diagnostics
        self.timer = self.create_timer(2.0, self.run_diagnostics)
        
        self.get_logger().info("=== ROVER DIAGNOSTICS STARTED ===")
        self.get_logger().info("Monitoring topics and controllers...")
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel_received = True
        self.last_cmd_vel = msg
        self.get_logger().info(f"✓ CMD_VEL: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")
        
    def drive_cmd_callback(self, msg):
        self.drive_cmd_received = True
        self.last_drive_cmd = msg
        speeds = [f"{speed:.3f}" for speed in msg.data]
        self.get_logger().info(f"✓ DRIVE_CMD: wheels=[{', '.join(speeds)}]")
        
    def joint_state_callback(self, msg):
        self.joint_states_received = True
        self.last_joint_states = msg
        if len(msg.velocity) >= 6:
            velocities = [f"{vel:.3f}" for vel in msg.velocity[:6]]
            self.get_logger().info(f"✓ JOINT_STATES: velocities=[{', '.join(velocities)}]")
    
    def run_diagnostics(self):
        self.get_logger().info("\n=== DIAGNOSTIC REPORT ===")
        
        # Check topic status
        self.get_logger().info("Topic Status:")
        self.get_logger().info(f"  /cmd_vel: {'✓ ACTIVE' if self.cmd_vel_received else '✗ NO DATA'}")
        self.get_logger().info(f"  /drive_controller/commands: {'✓ ACTIVE' if self.drive_cmd_received else '✗ NO DATA'}")
        self.get_logger().info(f"  /joint_states: {'✓ ACTIVE' if self.joint_states_received else '✗ NO DATA'}")
        
        # Check controller status
        self.check_controllers()
        
        # Reset flags for next cycle
        self.cmd_vel_received = False
        self.drive_cmd_received = False
        self.joint_states_received = False
        
    def check_controllers(self):
        if not self.controller_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("  Controller Manager: ✗ SERVICE NOT AVAILABLE")
            return
            
        request = ListControllers.Request()
        future = self.controller_list_client.call_async(request)
        
        # Non-blocking check
        if future.done():
            try:
                response = future.result()
                self.get_logger().info("Controller Status:")
                
                drive_controller_found = False
                steer_controller_found = False
                
                for controller in response.controller:
                    status = "✓ ACTIVE" if controller.state == "active" else f"✗ {controller.state.upper()}"
                    self.get_logger().info(f"  {controller.name}: {status}")
                    
                    if controller.name == "drive_controller":
                        drive_controller_found = True
                    elif controller.name == "steer_controller":
                        steer_controller_found = True
                
                if not drive_controller_found:
                    self.get_logger().error("  ✗ DRIVE_CONTROLLER NOT FOUND!")
                if not steer_controller_found:
                    self.get_logger().error("  ✗ STEER_CONTROLLER NOT FOUND!")
                    
            except Exception as e:
                self.get_logger().error(f"  Controller check failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    diagnostics = RoverDiagnostics()
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        diagnostics.get_logger().info("Diagnostics stopped by user")
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()