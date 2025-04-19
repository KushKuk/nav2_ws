#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    drive_share_dir = get_package_share_directory('drive')
    ros_ign_share_dir = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(drive_share_dir, 'models', 'drive', 'urdf', 'drive.urdf')
    world_path = os.path.join(drive_share_dir, 'worlds', 'maze.sdf')

    return LaunchDescription([
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_ign_share_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_path}'}.items()
        ),

        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Delay the spawn to ensure Gazebo is ready
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'drive',
                        '-topic', 'robot_description',
                        '-x', '0', '-y', '0', '-z', '0.7'
                    ],
                    output='screen'
                )
            ]
        )
    ])
