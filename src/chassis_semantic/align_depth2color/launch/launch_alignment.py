from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    orbbec_camera_dir = os.path.join(get_package_share_directory('orbbec_camera'),'launch','gemini_330_series.launch.py')
    launch_orbbec_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_dir),
        launch_arguments={
            'align_mode': 'HW_MODE',
            'align_target_stream': 'COLOR',
            'enable_d2c_viewer': 'false',

            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',

            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
        }.items()
    )

    launch_alignment = Node(
        package="align_depth2color",
        executable="alignment",
        name="align_depth2color",
        output='screen',
        parameters=[{
            "image_topic":"/camera/color/image_raw",
            "depth_topic":"/camera/depth/image_raw",
            "camera_info_topic":"/camera/color/camera_info",
            "output_topic":"/camera/aligned_depth_to_color/image_raw"
        }]
    )
    return LaunchDescription([
        launch_orbbec_driver,
        # launch_alignment
    ])