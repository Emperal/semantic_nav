from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction,ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    livox_ros_driver_path = get_package_share_directory('livox_ros_driver2')
    slam_path = get_package_share_directory('pc_to_scan_and_slam')
    launch_lidar_msgs = os.path.join(livox_ros_driver_path, 'launch_ROS2', 'msg_MID360s_launch.py')
    lio_sam_path = get_package_share_directory('lio_sam_mid360')
    launch_lio_sam = os.path.join(lio_sam_path,'launch','mapping_for_mobile.py')
    launch_slam = os.path.join(slam_path,"launch","pointcloud_to_scan_and_slam.launch.py")
    launch_chassis_driver_path = os.path.join(get_package_share_directory('dt_ros2'),'launch','dt_ros2.launch.py')
    launch_slam_toolbox_path = os.path.join(get_package_share_directory('pc_to_scan_and_slam'),'launch','pointcloud_to_scan_and_slam.launch.py')
    launch_nav_path = os.path.join(get_package_share_directory('chassis_nav'),'launch','navigation2.launch.py')
    rviz_config_dir = os.path.join(get_package_share_directory('pc_to_scan_and_slam'),'rviz','rviz.rviz')
    scan_filter_config_path = os.path.join(get_package_share_directory("pc_to_scan_and_slam"),"config","scan_self_filter.yaml")
    orbbec_camera_dir = os.path.join(get_package_share_directory('orbbec_camera'),'launch','gemini_330_series.launch.py')

    run_script = ExecuteProcess(
        cmd=['bash', '/home/robot2/hzf/1asemantic_nav/tum_dataset_generator.sh'],
        output='screen'
    )
    launch_lidar_msgs_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_lidar_msgs)
    )

    launch_lio_sam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_lio_sam)
    )

    launch_chassis_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_chassis_driver_path)
    )

    launch_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_slam_toolbox_path)
    )

    launch_orbbec_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_dir)
    )
    lidar_and_laser_filter_node = Node(
        package="pc_to_scan_and_slam",
        executable="laser_rect_filter_node",
        name="laser_rect_filter",
        output="screen",
        parameters=[
            scan_filter_config_path
        ]
    )

    launch_in_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[
            {"use_sim_time": False}
        ],
        output="screen"
    )

    # launch_nav = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_nav_path)
    # )

    # delay_launch_transform_node = TimerAction(
    #     period=3.0,
    #     actions=[launch_transform_node]
    # )
    # launch_slam_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_slam)
    # )

    return LaunchDescription([
        launch_chassis_driver,
        launch_in_rviz2,
        launch_lidar_msgs_node,
        run_script,
        # launch_orbbec_driver,
        TimerAction(period=3.0,actions=[lidar_and_laser_filter_node,launch_lio_sam_node]),
        # TimerAction(period=8.0,actions=[launch_slam_toolbox]),
        # TimerAction(period=9.0,actions=[launch_nav])
        # launch_slam_node
    ])
