from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    livox_ros_driver_path = get_package_share_directory('livox_ros_driver2')
    slam_path = get_package_share_directory('pc_to_scan_and_slam')
    launch_lidar_msgs = os.path.join(livox_ros_driver_path, 'launch_ROS2', 'msg_MID360s_launch.py')
    lio_sam_path = get_package_share_directory('lio_sam_mid360')
    launch_lio_sam = os.path.join(lio_sam_path,'launch','mapping_for_mobile.py')
    launch_slam = os.path.join(slam_path,"launch","pointcloud_to_scan_and_slam.launch.py")
    launch_chassis_driver_path = os.path.join(get_package_share_directory('dt_ros2'),'launch','dt_ros2.launch.py')
    launch_slam_toolbox_path = os.path.join(get_package_share_directory('pc_to_scan_and_slam'),'launch','pc_to_scan_and_filter.launch.py')
    launch_nav_path = os.path.join(get_package_share_directory('chassis_nav'),'launch','navigation2_3d.py')
    ekf_filter_path = os.path.join(get_package_share_directory('ekf_filter'),'launch/ekf_filter.launch.py')
    pub_wheel_odom = os.path.join(get_package_share_directory('ekf_filter'),'launch','publish_wheel_odometry.py')
    alignment_path = os.path.join(get_package_share_directory('pc2map_matcher'),'launch','pc2map_matcher.py')
    # pub_initial_pose_path = os.path.join(get_package_share_directory('scan_map_matcher'),'launch','scan_map_matcher.launch.py')
    ndt_localization = os.path.join(get_package_share_directory('ndt_localization'),'launch','launch_ndt_localization_node.py')
    config_dir = get_package_share_directory('chassis_nav')
    orbbec_camera_dir = os.path.join(get_package_share_directory('align_depth2color'),'launch','launch_alignment.py')
    nav_rpc_server_launch_path = os.path.join(get_package_share_directory('bson_rpc'),'launch','rpc_server_launch.py')
    semantic_nav_launch_path = os.path.join(get_package_share_directory('semantic_object_nav_rpc'),'launch','semantic_object_nav_rpc.launch.py')

    static_camera_to_livox_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_livox_to_camera_tf',
    arguments=[
        '-0.2510965414195721', '-0.049662425103220345', '0.6447900012558082', #x,y,z
        '0.01253724981247296', '-0.04784505468572336', '0.005874580850831631', '0.9987588084262821', #qx,qy,qz,qw
        'livox_frame',  #parent frame
        'camera_link' # child frame
        ]
    )

    launch_lidar_msgs_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_lidar_msgs)
    )

    launch_chassis_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_chassis_driver_path)
    )

    launch_orbbec_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_dir)
    )

    launch_scan_and_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_slam_toolbox_path)
    )

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_nav_path)
    )

    launch_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_filter_path)
    )

    launch_wheel_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pub_wheel_odom)
    )

    launch_nav_rpc_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_rpc_server_launch_path)
    )

    launch_semantic_nav =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(semantic_nav_launch_path)
    )
    # launch_supervisor = Node(
    #     package='nav_supervisor',
    #     executable='nav_supervisor',
    #     name='nav_supervisor_node',
    #     output='screen',
    # )

    launch_alignment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(alignment_path)
    )

    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ndt_localization)
    )

    # pub_initial_pose = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(pub_initial_pose_path)
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
        TimerAction(period=3.0,actions=[launch_lidar_msgs_node,launch_wheel_odom]),
        TimerAction(period=5.0,actions=[launch_scan_and_filter]),
        TimerAction(period=7.0,actions=[launch_nav]),
        TimerAction(period=10.0,actions=[launch_alignment,
                                        launch_localization
                                        ]),
        # TimerAction(period=12.0,actions=[launch_orbbec_driver,static_camera_to_livox_tf]),
        TimerAction(period=12.0,actions=[launch_nav_rpc_server,launch_semantic_nav]),
    ])
