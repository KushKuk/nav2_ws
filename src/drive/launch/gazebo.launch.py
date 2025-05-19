#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    drive_share_dir = get_package_share_directory('drive')
    ros_ign_share_dir = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(drive_share_dir, 'models', 'drive', 'urdf', 'drive.urdf')
    world_path = os.path.join(drive_share_dir, 'worlds', 'maze.sdf')

    # Read URDF safely
    try:
        with open(urdf_path, 'r') as urdf_file:
            robot_description_content = urdf_file.read()
    except FileNotFoundError:
        print(f"[ERROR] URDF file not found at: {urdf_path}")
        robot_description_content = ""

    return LaunchDescription([
        LogInfo(msg='🚀 Launching Ignition Gazebo with custom world...'),

        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_ign_share_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_path}'}.items()
        ),

        LogInfo(msg='🦾 Starting robot_state_publisher...'),

        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),

        LogInfo(msg='🕒 Waiting for Gazebo to be ready before spawning the robot...'),

        # Delay the spawn to ensure Gazebo is ready
        TimerAction(
            period=3.0,
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
        ),

        LogInfo(msg='📡 Launching IMU bridge...'),

        # IMU Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=['/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
            remappings=[('/imu/data', '/imu/data')],
            output='screen'
        ),

        LogInfo(msg='📡 Launching LiDAR bridge...'),

        # LiDAR Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            remappings=[('/scan', '/scan')],
            output='screen'
        ),
    ])
