from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to URDF, controller config, and world file
    drive_share_dir = get_package_share_directory('drive')
    ros_ign_share_dir = get_package_share_directory('ros_gz_sim')
    
    urdf_path = os.path.join(drive_share_dir, 'models', 'drive', 'urdf', 'drive.urdf')
    controller_yaml = PathJoinSubstitution([
        FindPackageShare('drive'),
        'models', 'drive', 'config', 'rover_controllers.yaml'
    ])
    world_path = os.path.join(drive_share_dir, 'worlds', 'maze.sdf')

    # Ignition Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_ign_share_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': open(urdf_path).read()}],
        output='screen'
    )

    # Spawn rover with delay
    spawn_rover = TimerAction(
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
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # IMU bridge
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=['/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
        remappings=[('/imu/data', '/imu/data')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # LiDAR bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        remappings=[('/scan', '/scan')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ROS2 control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': True, 'robot_description': open(urdf_path).read()}, controller_yaml],
        output='screen'
    )

    # Spawn drive controller
    spawn_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_controller'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn steer controller
    spawn_steer = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steer_controller'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Ackermann command converter
    ackermann_node = Node(
        package='differential_steering',
        executable='ackermann_cmd_vel_converter',
        name='ackermann_cmd_vel_converter',
        parameters=[{'use_sim_time': True}, {
            'wheel_radius': 0.1125,
            'max_steering_angle': 1.047,
            'robot_length': 1.0,
            'robot_width': 0.54
        }],
        output='screen'
    )

    # Keyboard teleoperation node
    teleop_node = Node(
        package='drive',
        executable='drive_node',
        name='drive_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_rover,
        imu_bridge,
        lidar_bridge,
        ros2_control_node,
        TimerAction(period=2.0, actions=[spawn_drive, spawn_steer]),
        ackermann_node,
        teleop_node
    ])