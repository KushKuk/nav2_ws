#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    drive_share_dir = get_package_share_directory('drive')
    ros_ign_share_dir = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(drive_share_dir, 'models', 'drive', 'urdf', 'drive.urdf')
    world_path = os.path.join(drive_share_dir, 'worlds', 'maze.sdf')
    controller_config = os.path.join(drive_share_dir, 'models', 'drive', 'config', 'rover_controllers.yaml')

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

        # IMU Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=['/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
            remappings=[('/imu/data', '/imu/data')],
            output='screen'
        ),

        # LiDAR bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            remappings=[('/scan', '/scan')],
            output='screen'
        ),

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
                '-x', '0', '-y', '0', '-z', '1',
                '-R', '0', '-P', '3.14', '-Y', '0'
            ],
            output='screen'
        )
    ]


        ),

        # ROS2 Control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drive_share_dir, 'launch', 'ros2_control.launch.py')
            )
        ),
        
        # Differential Steering Converter - CORE FUNCTIONALITY
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='differential_steering',
                    executable='ackermann_cmd_vel_converter',
                    name='ackermann_cmd_vel_converter',
                    parameters=[
                        {'wheel_radius': 0.1125},           
                        {'max_steering_angle': 1.047},      
                        {'robot_length': 1.0},              
                        {'robot_width': 0.54}               
                    ],
                    output='screen',
                    remappings=[
                        ('/cmd_vel', '/cmd_vel'),                              
                        ('/drive_controller/commands', '/drive_controller/commands'),  
                        ('/steer_controller/commands', '/steer_controller/commands')   
                    ]
                )
            ]
        ),


        # Status Monitor - provides feedback WITHOUT sending commands
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='drive',
                    executable='rover_status_monitor.py',
                    name='rover_status_monitor',
                    output='screen'
                )
            ]
        ),
    ])