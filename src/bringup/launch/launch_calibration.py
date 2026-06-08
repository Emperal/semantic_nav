from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command
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
    orbbec_camera_dir = os.path.join(get_package_share_directory('orbbec_camera'),'launch','gemini_330_series.launch.py')
    xacro_path = os.path.join(get_package_share_directory("chassis_description"), "urdf")
    rviz_config_file = os.path.join(get_package_share_directory('bringup'),'rviz','rviz.rviz')
    
    static_tf_cam_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_lidar_tf',
        arguments=[
            '0.1', '0.0', '0.2',   # x y z（单位：米）
            '0', '0', '0',         # roll pitch yaw（单位：弧度）
            'livox_frame',           # 父坐标系 雷达坐标系
            'camera_link'         # 子坐标系 相机坐标系
        ]
    )

    robot_state = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            "xacro",
                            " ",
                            xacro_path,
                            "/",
                            "chassis_description",
                            ".urdf",
                        ]
                    ),
                    value_type=str,
                )
            }
        ],
        )

    launch_orbbec_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_dir),
        launch_arguments={
            # 'align_mode': 'HW_MODE',
            # 'align_target_stream': 'COLOR',
            # 'enable_d2c_viewer': 'true',

            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',

            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
        }.items()
    )
    launch_lidar_msgs_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_lidar_msgs)
    )

    sensor_calibration_manager_node = Node(
            package='sensor_calibration_manager',
            executable='sensor_calibration_manager',
            name='sensor_calibration_manager',
            output='screen'
    )

    c2l_cali_node = Node(
        package='lidar_camera_manual_calib',
        executable='manual_pnp_calibrator',
        name='manual_pnp_calibrator',
        output='screen',

        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'clicked_point_topic': '/clicked_point',

            'camera_link_frame': 'camera_link',
            'camera_frame': 'camera_color_optical_frame',
            'lidar_frame': 'livox_frame',

            'save_path': '/home/robot2/hzf/2D_robot_ros2_final/src/bringup/calibration_res/c2l_cali_res.yaml',
        }]
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

#     ros2 run lidar_camera_manual_calib manual_pnp_calibrator --ros-args \
#   -p image_topic:=/camera/color/image_raw \
#   -p camera_info_topic:=/camera/color/camera_info \
#   -p clicked_point_topic:=/clicked_point \
#   -p lidar_frame:=livox_frame \
#   -p camera_frame:=camera_color_optical_frame \
#   -p save_path:=/home/robot2/hzf/lidar_camera_extrinsic.yaml

    delayed_calibrator = TimerAction(
        period=4.0,
        actions=[c2l_cali_node]
    )

    #ros2 run sensor_calibration_manager sensor_calibration_manager

    return LaunchDescription([
        launch_lidar_msgs_node,launch_orbbec_driver,
        # sensor_calibration_manager_node,
        start_rviz,
        static_tf_cam_lidar,
        delayed_calibrator,
    ])
