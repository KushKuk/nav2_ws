import launch
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_model_path = PathJoinSubstitution(
        [FindPackageShare('drive_nav2'), 'urdf', 'drive.urdf']
    )

    import os
    from ament_index_python.packages import get_package_share_directory
    
    urdf_path = os.path.join(get_package_share_directory('drive_nav2'), 'urdf', 'drive.urdf')
    robot_description_content = os.popen(f'xacro {urdf_path}').read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file',
        ),
        robot_state_publisher_node,
    ])
