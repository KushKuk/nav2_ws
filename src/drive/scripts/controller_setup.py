#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, LoadController, ConfigureController, SwitchController
import time

class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager_helper')
        
        # Service clients
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.load_client = self.create_client(LoadController, '/controller_manager/load_controller')
        self.configure_client = self.create_client(ConfigureController, '/controller_manager/configure_controller')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        
        self.get_logger().info("=== CONTROLLER MANAGER HELPER ===")
        
    def wait_for_services(self):
        """Wait for all controller manager services"""
        services = [
            (self.list_client, 'list_controllers'),
            (self.load_client, 'load_controller'),
            (self.configure_client, 'configure_controller'),
            (self.switch_client, 'switch_controller')
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service {name} not available!")
                return False
        return True
    
    def list_controllers(self):
        """List all controllers and their status"""
        if not self.list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("List controllers service not available")
            return []
        
        request = ListControllers.Request()
        future = self.list_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info("Current Controllers:")
            for controller in response.controller:
                self.get_logger().info(f"  - {controller.name}: {controller.state}")
            return response.controller
        else:
            self.get_logger().error("Failed to list controllers")
            return []
    
    def load_controller(self, controller_name):
        """Load a controller"""
        request = LoadController.Request()
        request.name = controller_name
        
        future = self.load_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            success = future.result().ok
            if success:
                self.get_logger().info(f"✓ Loaded controller: {controller_name}")
            else:
                self.get_logger().error(f"✗ Failed to load controller: {controller_name}")
            return success
        else:
            self.get_logger().error(f"✗ Load controller service call failed for: {controller_name}")
            return False
    
    def configure_controller(self, controller_name):
        """Configure a controller"""
        request = ConfigureController.Request()
        request.name = controller_name
        
        future = self.configure_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            success = future.result().ok
            if success:
                self.get_logger().info(f"✓ Configured controller: {controller_name}")
            else:
                self.get_logger().error(f"✗ Failed to configure controller: {controller_name}")
            return success
        else:
            self.get_logger().error(f"✗ Configure controller service call failed for: {controller_name}")
            return False
    
    def start_controller(self, controller_name):
        """Start/activate a controller"""
        request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            success = future.result().ok
            if success:
                self.get_logger().info(f"✓ Started controller: {controller_name}")
            else:
                self.get_logger().error(f"✗ Failed to start controller: {controller_name}")
            return success
        else:
            self.get_logger().error(f"✗ Start controller service call failed for: {controller_name}")
            return False
    
    def setup_controllers(self):
        """Setup all required controllers"""
        self.get_logger().info("Setting up controllers...")
        
        if not self.wait_for_services():
            return False
        
        # List current controllers
        controllers = self.list_controllers()
        
        # Required controllers
        required_controllers = ['drive_controller', 'steer_controller', 'joint_state_broadcaster']
        
        for controller_name in required_controllers:
            # Check if controller exists
            controller_exists = any(c.name == controller_name for c in controllers)
            
            if not controller_exists:
                self.get_logger().info(f"Loading {controller_name}...")
                if not self.load_controller(controller_name):
                    continue
            
            # Check if controller is configured
            controller_state = next((c.state for c in controllers if c.name == controller_name), 'unknown')
            
            if controller_state in ['unconfigured', 'unknown']:
                self.get_logger().info(f"Configuring {controller_name}...")
                if not self.configure_controller(controller_name):
                    continue
            
            # Check if controller is active
            if controller_state != 'active':
                self.get_logger().info(f"Starting {controller_name}...")
                self.start_controller(controller_name)
        
        # Final status check
        self.get_logger().info("\nFinal controller status:")
        self.list_controllers()
        return True

def main(args=None):
    rclpy.init(args=args)
    manager = ControllerManager()
    
    try:
        success = manager.setup_controllers()
        if success:
            manager.get_logger().info("=== CONTROLLER SETUP COMPLETED ===")
        else:
            manager.get_logger().error("=== CONTROLLER SETUP FAILED ===")
    except KeyboardInterrupt:
        manager.get_logger().info("Controller setup interrupted by user")
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()