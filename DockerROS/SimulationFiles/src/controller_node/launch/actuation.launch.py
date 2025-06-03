import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    # Run the node
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        Node(
            package='controller_node',
            executable='simple_pid',
            name='simple_pid'),
    ])