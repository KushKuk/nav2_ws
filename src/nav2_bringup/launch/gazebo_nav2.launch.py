import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_launch_path = os.path.join(
        get_package_share_directory('drive'),
        'launch',
        'gazebo.launch.py'  # Or your Gazebo launch file
    )

    robot_model_path = os.path.join(
        get_package_share_directory('drive'),
        'models',
        'drive',
        'drive.urdf'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[robot_model_path],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_link'],
        ),
        # Add more static transform publishers for other sensor frames if needed
    ])
