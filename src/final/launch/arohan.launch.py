import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

def generate_launch_description():
    pkg_share= launch_ros.substituions.FindPackageShare(package="drive").find("drive")
    default_model_path=os.path.join(pkg_share,"src/drive/models/drive/urdf/drive.urdf")
    default_rviz_config_path =os.path.join
