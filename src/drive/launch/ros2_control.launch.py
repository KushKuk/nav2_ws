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
    ])
