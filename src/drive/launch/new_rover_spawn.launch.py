#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package Directories
    pkg_drive = FindPackageShare(package='drive').find('drive')
    
    # Set paths
    default_model_path = os.path.join(pkg_drive, 'models')
    default_world_path = os.path.join(pkg_drive, 'worlds')
    urdf_file = os.path.join(pkg_drive, 'models', 'drive', 'urdf', 'drive.urdf')
    world_file = os.path.join(pkg_drive, 'worlds', 'maze.sdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='drive_rover',
        description='Name of the robot')
    
    declare_spawn_x_cmd = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='x position to spawn the robot')
    
    declare_spawn_y_cmd = DeclareLaunchArgument(
        'spawn_y', 
        default_value='-8.0',
        description='y position to spawn the robot (start area)')
    
    declare_spawn_z_cmd = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.5',
        description='z position to spawn the robot')
    
    declare_spawn_yaw_cmd = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='1.57',
        description='yaw angle to spawn the robot (facing forward)')

    # Read robot description from URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Start Gazebo with the maze world
    start_gazebo_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-v', '4'],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot_cmd = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to start
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', robot_name,
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                    '-Y', spawn_yaw
                ],
                output='screen'
            )
        ]
    )

    # Bridge between ROS and Gazebo for essential topics
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_spawn_x_cmd)
    ld.add_action(declare_spawn_y_cmd)
    ld.add_action(declare_spawn_z_cmd)
    ld.add_action(declare_spawn_yaw_cmd)

    # Add the actions to launch all nodes
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(ros_gz_bridge)

    return ld