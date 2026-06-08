# from launch import LaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription
# from launch.actions import TimerAction
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     orbbec_camera_dir = os.path.join(get_package_share_directory('orbbec_camera'),'launch','gemini_330_series.launch.py')
#     launch_orbbec_driver = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(orbbec_camera_dir)
#     )
#     # launch_qwen_nav_pkg = Node(
#     #     package='qwen_nav_pkg',
#     #     executable='qwen_nav_node',
#     #     name='qwen_nav_node',
#     #     output='screen',
#     # )

#     return LaunchDescription([
#         launch_orbbec_driver,#launch_qwen_nav_pkg
#     ])

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
        PythonLaunchDescriptionSource(orbbec_camera_dir)
    )

    launch_alignment = Node(
        package="align_depth2color",
        executable="alignment",
        name="align_depth2color",
        output='screen',
        parameters=[{
            "image_topic":"/camera/color/image_raw",
            "depth_toipc":"/camera/depth/image_raw",
            "camera_info_toipc":"/camera/color/camera_info",
            "output_topic":"/camera/aligned_depth_to_color/image_raw"
        }]
    )
    return LaunchDescription([
        launch_orbbec_driver,launch_alignment
    ])