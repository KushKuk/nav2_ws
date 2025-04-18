#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package paths
    drive_share_dir = get_package_share_directory('drive')
    ros_ign_share_dir = get_package_share_directory('ros_ign_gazebo')
    
    # URDF path
    urdf_path = os.path.join(drive_share_dir, 'models', 'drive', 'urdf', 'drive.urdf')
    
    # World file path
    world_path = os.path.join(drive_share_dir, 'worlds', 'empty.world')
    
    return LaunchDescription([
        # Launch Ignition Gazebo with the empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_ign_share_dir, 'launch', 'ign_gazebo.launch.py')
            ),
            launch_arguments={
                'ign_args': f'-r {world_path}'
            }.items()
        ),
        
        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Spawn the robot in Ignition
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-name', 'drive',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '1.0'
            ],
            output='screen'
        ),
    ])