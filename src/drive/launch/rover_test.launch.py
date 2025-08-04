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

        # Spawn robot
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
        
        # Differential Steering Converter
        TimerAction(
            period=8.0,
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
                    output='screen'
                )
            ]
        ),

        # Diagnostics - starts immediately to monitor everything
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='drive',
                    executable='rover_diagnostics.py',
                    name='rover_diagnostics',
                    output='screen'
                )
            ]
        ),

        # Movement Test - starts after diagnostics
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal', '--', 'bash', '-c', 
                        'source /home/ujjwal/nav2_ws/install/setup.bash && '
                        'echo "=== ROVER MOVEMENT TEST ===" && '
                        'echo "This will test rover movement automatically" && '
                        'echo "Watch the diagnostics terminal for status" && '
                        'echo "Press ENTER to start movement test..." && read && '
                        'ros2 run drive rover_movement_test.py'
                    ],
                    output='screen'
                )
            ]
        ),

        # Manual Control Option
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal', '--', 'bash', '-c', 
                        'source /home/ujjwal/nav2_ws/install/setup.bash && '
                        'echo "=== MANUAL ROVER CONTROL ===" && '
                        'echo "Use these keys to control the rover:" && '
                        'echo "  w/x: forward/backward" && '
                        'echo "  a/d: turn left/right" && '
                        'echo "  s: stop" && '
                        'echo "  CTRL+C: exit" && '
                        'echo "" && '
                        'echo "Press ENTER to start manual control..." && read && '
                        'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel'
                    ],
                    output='screen'
                )
            ]
        ),
    ])