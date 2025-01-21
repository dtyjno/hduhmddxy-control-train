
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from pathlib import Path

def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_ros_gz_sim_ardupilot = get_package_share_directory('ros_gz_sim_ardupilot')
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Gazebo.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r iris_runway.sdf'}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            gz_sim,
        ]
    )
