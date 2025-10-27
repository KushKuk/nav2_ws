#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    drive_share_dir = get_package_share_directory('drive')
    
    # Configuration files
    slam_params_file = os.path.join(drive_share_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(drive_share_dir, 'config', 'slam.rviz')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # RViz2 Node (optional - uncomment when you have slam.rviz config)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen',
    # )

    # Create launch description
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_use_sim_time_arg)

    # Add nodes
    ld.add_action(slam_toolbox_node)
    # ld.add_action(rviz_node)  # Uncomment when you have slam.rviz config

    return ld