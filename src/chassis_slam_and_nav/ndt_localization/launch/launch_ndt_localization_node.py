from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    pcd_file_path = os.path.join(get_package_share_directory('pc2map_matcher'),'map','cloudGlobal.pcd')
    yaml_file_path = os.path.join(get_package_share_directory('chassis_nav'),'maps','room.yaml')
    pgm_file_path = os.path.join(get_package_share_directory('chassis_nav'),'maps','room.pgm')
    return LaunchDescription([
        Node(
            package='ndt_localization',
            executable='ndt_localization_node',
            parameters=[
                {'map_path':pcd_file_path},
                ]
        )
    ])