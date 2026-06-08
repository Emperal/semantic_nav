import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='nav_sdk',
            executable='nav_rpc_server',
            name='nav_rpc_server',
            output='screen',
            # remappings=[
            #     ('/odometry/filtered', '/odom')
            # ]
        )
    ])