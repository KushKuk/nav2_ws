#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_ros_gz_sim = get_package_share_directory('drive')
    pkg_drive = get_package_share_directory('drive')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    world_file = LaunchConfiguration('world_file')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('drive'),
            'worlds',
            'maze.sdf'
        ]),
        description='Path to world file'
    )
    
    # Paths to important files
    urdf_file = os.path.join(pkg_drive, 'models', 'drive', 'urdf', 'drive.urdf')

    rviz_config = os.path.join(pkg_drive, 'rviz', 'drive_rover.rviz')
    controllers_config = os.path.join(pkg_drive, 'models', 'drive', 'config', 'rover_controllers.yaml')
    
    # Set Gazebo model path
    set_env_var = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_drive, 'models')
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file).read()
        }]
    )
    

    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'drive_rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',      # Much higher spawn height to clear wheels from ground
            '-R', '0',  # Roll rotation to fix orientation
            '-P', '0.0',      # Pitch
            '-Y', '0.0'       # Yaw
        ],
        output='screen'
    )
    
    # Bridge between Gazebo and ROS topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('drive', 'launch', 'ros2_control.launch.py')
            )
        ),
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Load and start controllers
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    Plugin = Node(
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
            ('/drive_controller/commands', '/drive_controller/commands')  
            ]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_world_file,
        set_env_var,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        controller_manager,
        load_joint_state_controller,
        load_diff_drive_controller,
        rviz
    ])
