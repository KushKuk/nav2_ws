from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                os.path.join(
                    os.getenv('HOME'), 'nav2_ws/src/drive/models/drive/urdf/drive.urdf'
                ),
                os.path.join(
                    os.getenv('HOME'), 'nav2_ws/src/drive/models/drive/config/rover_controllers.yaml'
                )
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['drive_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['steer_controller'],
            output='screen',
        ),
        
        # Ackermann Command Velocity Converter for 6-Wheel Rover
        Node(
            package='differential_steering',
            executable='ackermann_cmd_vel_converter',
            name='ackermann_cmd_vel_converter',
            parameters=[
                {'wheel_radius': 0.1125},           # Wheel radius from URDF
                {'max_steering_angle': 1.047},      # 60 degrees max steering
                {'robot_length': 1.0},              # Distance between front/rear axles
                {'robot_width': 0.54}               # Distance between left/right wheels
            ],
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel'),                              # Input: velocity commands
                ('/drive_controller/commands', '/drive_controller/commands'),  # Output: wheel speeds
                ('/steer_controller/commands', '/steer_controller/commands')   # Output: steering angles
            ]
        ),
    ])
