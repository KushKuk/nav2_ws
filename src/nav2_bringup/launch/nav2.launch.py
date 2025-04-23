import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='nav2_controller',
            name='controller',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_planner',
            executable='nav2_planner',
            name='planner',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_tree_engine',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_recoveries',
            executable='nav2_recoveries',
            name='recoveries',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True}, {'autostart': True},
                        {'node_names': ['map_server', 'amcl', 'planner',
                                        'controller', 'bt_navigator', 'recoveries']}]
        )
    ])
