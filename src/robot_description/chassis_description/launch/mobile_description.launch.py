import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.launch_context import LaunchContext
from typing import List
from launch_ros.parameter_descriptions import ParameterValue


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


ARGUMENTS = [
    DeclareLaunchArgument(
        "model", default_value="chassis_description", description="Robot Model"
    )
]


def generate_launch_description():

    xacro_path = os.path.join(get_package_share_directory("chassis_description"), "urdf")
    rviz_config_path = os.path.join(
        get_package_share_directory("chassis_description"), "rviz", "urdf_config.rviz"
    )
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "chassis_base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            "xacro",
                            " ",
                            xacro_path,
                            "/",
                            LaunchConfiguration("model"),
                            ".urdf",
                        ]
                    ),
                    value_type=str,
                )
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    return LaunchDescription(
        ARGUMENTS + [static_tf, robot_state_publisher, joint_state_publisher, rviz_node]
    )
