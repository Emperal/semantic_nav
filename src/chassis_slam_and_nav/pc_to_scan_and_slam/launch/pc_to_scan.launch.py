from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    
    pc_to_scan_path = get_package_share_directory("pc_to_scan_and_slam")
    pc_to_scan_config_path = os.path.join(pc_to_scan_path,"config","pointcloud_to_scan.yaml")
    slam_toolbox_config_path = os.path.join(pc_to_scan_path,"config","slam_toolbox.yaml")
    map_pkg_path = get_package_share_directory('chassis_nav')
    map_yaml = os.path.join(map_pkg_path, 'maps', 'room.yaml')


    pc_to_scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',

            parameters=[
                {'use_sim_time': False},
               pc_to_scan_config_path
            ],

            remappings=[
                ('cloud_in', '/livox/lidar'),

                ('scan', '/scan')
            ]
    )

    return LaunchDescription([
        pc_to_scan,
    ])
