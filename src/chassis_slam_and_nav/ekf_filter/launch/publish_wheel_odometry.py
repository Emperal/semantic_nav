import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ekf_filter',
            executable='odom_to_tf_node',
            name='publish_odom_tf',
            output='screen',
        )
    ])