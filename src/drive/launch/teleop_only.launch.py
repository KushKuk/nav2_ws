#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Keyboard Teleop
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='gnome-terminal --',
            parameters=[],
            remappings=[
                ('cmd_vel', '/cmd_vel')
            ]
        ),
        
        # Optionally add joystick teleop (uncomment if you have a gamepad)
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     output='screen'
        # ),
        #
        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_twist_joy',
        #     output='screen',
        #     remappings=[
        #         ('cmd_vel', '/cmd_vel')
        #     ]
        # ),
    ])