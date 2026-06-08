from launch.substitutions import Command
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory("chassis_description"), "urdf")

    return LaunchDescription([
        Node(
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
        ),

        Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        ),
        Node(
            package='dt_ros2',
            executable='dt_ros2_node',
            name='dt_ros2_node',
            output='screen',
            parameters=[{
                'ip': '192.168.1.100',
                'port': 9999,
                'dt_odom_enable': True,
                'dt_drive_type': 'SDFZ', # 'SDFZ' of 'HLS'
                'dt_log_display': False,
                'dt_original_display': False,

            }],
            remappings=[
                # pub: old, new
                ('/dt/frame_info',             '/dt/frame_info'),
                ('/dt/drive_left_error_info',  '/dt/drive_left_error_info'),
                ('/dt/drive_right_error_info', '/dt/drive_right_error_info'),
                ('/dt/auto_charge_info',       '/dt/auto_charge_info'),
                ('/dt/battery_info',           '/dt/battery_info'),
                ('/dt/bms_info',               '/dt/bms_info'),
                ('/dt/date_info',              '/dt/date_info'),
                ('/dt/parameter_info',         '/dt/parameter_info'),
                ('/dt/state_info',             '/dt/state_info'),
                ('/dt/velocity_info',          '/dt/velocity_info'),
                ('/dt/hardware_version_info',  '/dt/hardware_version_info'),
                ('/dt/led_strip_mode_info',    '/dt/led_strip_mode_info'),
                ('/dt/led_strips_info',        '/dt/led_strips_info'),
                ('/dt/remote_control_info',    '/dt/remote_control_info'),
                ('/dt/software_version_info',  '/dt/software_version_info'),
                ('/dt/current_info',           '/dt/current_info'),
                ('/dt/speed_info',             '/dt/speed_info'),
                ('/dt/odom_info',              '/dt/odom_info'),
                ('/dt/battery_state',          '/dt/battery_state'),

                # sub: old, new
                ('/dt/velocity_ctrl',       '/cmd_vel'),
                ('/dt/speed_ctrl',          '/dt/speed_ctrl'),
                ('/dt/stop_ctrl',           '/dt/stop_ctrl'),
                ('/dt/collision_clean',     '/dt/collision_clean'),
                ('/dt/fault_clean',         '/dt/fault_clean'),
                ('/dt/auto_charge_ctrl',    '/dt/auto_charge_ctrl'),
                ('/dt/led_strip_mode_ctrl', '/dt/led_strip_mode_ctrl'),
                ('/dt/led_strip_ctrl',      '/dt/led_strip_ctrl'),
                ('/dt/led_strips_ctrl',     '/dt/led_strips_ctrl'),
                ('/dt/odom_clean',          '/dt/odom_clean'),
            ]
        )
    ])