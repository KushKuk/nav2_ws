#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import tempfile
import yaml


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    # Robot description - use the original URDF
    with open('/home/ujjwal/nav2_ws/src/drive/models/drive/urdf/drive.urdf', 'r') as file:
        robot_description_content = file.read()

    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    controller_config = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 100,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                "six_wheel_drive_controller": {
                    "type": "diff_drive_controller/DiffDriveController"
                }
            }
        },
        "six_wheel_drive_controller": {
            "ros__parameters": {
                "left_wheel_names": ["joint_wheel_1", "joint_wheel_3", "joint_wheel_5"],
                "right_wheel_names": ["joint_wheel_2", "joint_wheel_4", "joint_wheel_6"],
                "wheel_separation": 0.54,
                "wheel_radius": 0.1125,
                "wheel_separation_multiplier": 1.0,
                "left_wheel_radius_multiplier": 1.0,
                "right_wheel_radius_multiplier": 1.0,
                "publish_rate": 50.0,
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "pose_covariance_diagonal": [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
                "twist_covariance_diagonal": [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
                "open_loop": False,
                "enable_odom_tf": True,
                "cmd_vel_timeout": 0.5,
                "publish_limited_velocity": True,
                "use_stamped_vel": False,
                "velocity_rolling_window_size": 10,
                "linear.x.has_velocity_limits": True,
                "linear.x.has_acceleration_limits": True,
                "linear.x.has_jerk_limits": False,
                "linear.x.max_velocity": 1.0,
                "linear.x.min_velocity": -1.0,
                "linear.x.max_acceleration": 1.0,
                "linear.x.max_jerk": 0.0,
                "linear.x.min_jerk": 0.0,
                "angular.z.has_velocity_limits": True,
                "angular.z.has_acceleration_limits": True,
                "angular.z.has_jerk_limits": False,
                "angular.z.max_velocity": 1.0,
                "angular.z.min_velocity": -1.0,
                "angular.z.max_acceleration": 1.0,
                "angular.z.min_acceleration": -1.0,
                "angular.z.max_jerk": 0.0,
                "angular.z.min_jerk": 0.0
            }
        }
    }

    # Create a temporary config file
    config_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(controller_config, config_file, default_flow_style=False)
    config_file.close()

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Six wheel drive controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["six_wheel_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(start_rviz),
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # Note: We're not including the ros2_control_node here because it needs hardware interfaces
    # This launch file is for demonstration of the controller configuration

    nodes = [
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)


if __name__ == '__main__':
    generate_launch_description()