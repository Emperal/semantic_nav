from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)


def generate_launch_description():

    pc_to_scan_path = get_package_share_directory("pc_to_scan_and_slam")
    pc_to_scan_config_path = os.path.join(
        pc_to_scan_path,
        "config",
        "pointcloud_to_scan.yaml"
    )

    slam_toolbox_config_path = os.path.join(
        pc_to_scan_path,
        "config",
        "slam_toolbox.yaml"
    )

    scan_filter_config_path = os.path.join(
        get_package_share_directory("pc_to_scan_and_slam"),
        "config",
        "scan_self_filter.yaml"
    )

    rviz_config_dir = load_file(
        "pc_to_scan_and_slam",
        "rviz/rviz.rviz"
    )

    pc_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            pc_to_scan_config_path
        ],
        remappings=[
            ("cloud_in", "/livox/lidar"),
            ("scan", "/scan_origin")
        ]
    )

    # laser_filter_node = Node(
    #     package="pc_to_scan_and_slam",
    #     executable="laser_rect_filter_node",
    #     name="laser_rect_filter",
    #     output="screen",
    #     parameters=[
    #         scan_filter_config_path
    #     ]
    # )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_sync_launch.py"
            )
        ),
        launch_arguments={
            "slam_params_file": slam_toolbox_config_path,
            "use_sim_time": "false"
        }.items()
    )

    # launch_in_rviz2 = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config_dir],
    #     parameters=[
    #         {"use_sim_time": False}
    #     ],
    #     output="screen"
    # )

    return LaunchDescription([
        pc_to_scan,
        # laser_filter_node,
        slam_toolbox_launch,
        # launch_in_rviz2,
    ])