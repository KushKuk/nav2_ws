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
    slam_toolbox_share_dir = get_package_share_directory('slam_toolbox')

    urdf_path = os.path.join(drive_share_dir, 'models', 'drive', 'urdf', 'drive.urdf')
    world_path = os.path.join(drive_share_dir, 'worlds', 'maze.sdf')
    slam_params_path = os.path.join(drive_share_dir, 'config', 'slam_params.yaml')
    ekf_params_path = os.path.join(drive_share_dir, 'config', 'ekf.yaml')

    return LaunchDescription([

        # 1️⃣ Start Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_ign_share_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_path}'}.items()
        ),

        # 2️⃣ Clock bridge (fixed syntax)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/world/maze_world/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
            remappings=[('/world/maze_world/clock', '/clock')],
            output='screen'
        ),

        # 3️⃣ IMU bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=['/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 4️⃣ LiDAR bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # 5️⃣ Spawn robot after Ignition is ready
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'drive',
                        '-topic', 'robot_description',
                        '-x', '0', '-y', '0', '-z', '1'
                    ],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                )
            ]
        ),

        # 6️⃣ Start robot_state_publisher AFTER spawn
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[
                        {'robot_description': open(urdf_path).read()},
                        {'use_sim_time': True}
                    ],
                    output='screen'
                ),
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen'
                ),
            ]
        ),

        # 7️⃣ ROS 2 Control (controllers etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drive_share_dir, 'launch', 'ros2_control.launch.py')
            )
        ),

        # 8️⃣ Ackermann / Differential Steering Converter
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

        # 9️⃣ Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_path],
            remappings=[('/odometry/filtered', '/odom_filtered')]
        ),

        # 🔟 SLAM Toolbox
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_toolbox_share_dir, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'slam_params_file': slam_params_path,
                        'use_sim_time': 'True'
                    }.items(),
                )
            ]
        ),

        # 1️⃣1️⃣ Rover status monitor
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