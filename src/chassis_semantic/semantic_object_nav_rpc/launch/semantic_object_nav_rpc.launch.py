#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "npz_path",
            default_value="/home/robot2/hzf/1asemantic_nav/lib/pyslam/results/metrics_20260604_230221/semantic_dense_map_latest.npz",
        ),
        DeclareLaunchArgument("ply_path", default_value="/home/robot2/hzf/1asemantic_nav/lib/pyslam/results/metrics_20260604_230221/semantic_dense_map_latest.ply"),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("min_points", default_value="20"),
        DeclareLaunchArgument("ignore_class_ids", default_value="0"),
        DeclareLaunchArgument("approach_distance", default_value="0.8"),
        DeclareLaunchArgument("semantic_rpc_port", default_value="6001"),
        DeclareLaunchArgument("nav_rpc_port", default_value="6000"),

        Node(
            package="semantic_object_nav_rpc",
            executable="semantic_object_rpc_server",
            name="semantic_object_rpc_server",
            output="screen",
            parameters=[{
                "npz_path": LaunchConfiguration("npz_path"),
                "semantic_ply_path": LaunchConfiguration("ply_path"),
                "frame_id": LaunchConfiguration("frame_id"),
                "min_points": LaunchConfiguration("min_points"),
                "ignore_class_ids": "0",
                "approach_distance": LaunchConfiguration("approach_distance"),
                "semantic_rpc_port": LaunchConfiguration("semantic_rpc_port"),
                "nav_rpc_port": LaunchConfiguration("nav_rpc_port"),
            }],
        )
    ])
