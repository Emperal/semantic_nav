#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
semantic_db_nav_executor.py

配套 semantic_yoloe_db_runtime.py 使用的导航执行脚本。

使用场景：
1. SLAM 阶段：
   semantic_yoloe_db_runtime.py --mode build
   建立语义数据库。

2. 导航阶段：
   先启动本脚本：
       python3 semantic_db_nav_executor.py

   然后发布语义目标：
       python3 semantic_yoloe_db_runtime.py --mode goto --query "帮我找椅子"

   semantic_yoloe_db_runtime.py 会从数据库中查询椅子的 map 坐标，
   并发布到 /yoloe/target_pose。

   本脚本订阅 /yoloe/target_pose，
   计算一个距离目标物体 goal_distance 的导航点，
   然后通过 RPC 调用：
       server.add_goal(goal_x, goal_y, goal_yaw)

特点：
- 只做数据库语义目标导航，不做 YOLOE，不做搜索旋转。
- 输入目标点建议是 map 坐标。
- 如果输入目标点不是 map 坐标，也会尝试通过 TF 转到 map。
- 机器人不会直接撞到物体，而是停在目标前 goal_distance 米处。
"""

import math
import time
import socket
import argparse
import traceback

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from bsonrpc import JSONRpc

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def distance_2d(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    return math.sqrt(dx * dx + dy * dy)


class SemanticDatabaseNavExecutor(Node):
    def __init__(self, args):
        super().__init__("semantic_db_nav_executor")

        self.args = args

        # -----------------------------
        # 话题与坐标系
        # -----------------------------
        self.target_pose_topic = args.target_pose_topic
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame

        # -----------------------------
        # RPC 参数
        # -----------------------------
        self.rpc_host = args.rpc_host
        self.rpc_port = int(args.rpc_port)

        self.sock = None
        self.rpc = None
        self.server = None

        # -----------------------------
        # 导航参数
        # -----------------------------
        self.goal_distance = float(args.goal_distance)
        self.min_target_distance = float(args.min_target_distance)

        self.send_once = bool(args.send_once)

        self.send_interval = float(args.send_interval)
        self.min_goal_change = float(args.min_goal_change)

        self.target_timeout = float(args.target_timeout)
        self.tick_period = float(args.tick_period)

        # -----------------------------
        # 状态
        # -----------------------------
        self.latest_target_msg = None
        self.latest_target_time = 0.0

        self.last_sent_goal = None
        self.last_goal_send_time = 0.0

        self.sent_once_for_current_target = False

        # 用于判断是否来了一个新目标
        self.current_target_key = None

        # -----------------------------
        # TF
        # -----------------------------
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=20.0)
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self.target_pose_callback,
            10
        )

        self.timer = self.create_timer(
            self.tick_period,
            self.timer_callback
        )

        self.connect_rpc()

        self.get_logger().info("Semantic Database Nav Executor started.")
        self.get_logger().info(f"Subscribe target pose: {self.target_pose_topic}")
        self.get_logger().info(f"RPC server: {self.rpc_host}:{self.rpc_port}")
        self.get_logger().info(f"map_frame: {self.map_frame}")
        self.get_logger().info(f"base_frame: {self.base_frame}")
        self.get_logger().info(f"goal_distance: {self.goal_distance}")
        self.get_logger().info(f"send_once: {self.send_once}")

    # ============================================================
    # RPC
    # ============================================================

    def connect_rpc(self):
        self.close_rpc()

        try:
            self.get_logger().info(
                f"Connecting RPC server {self.rpc_host}:{self.rpc_port}"
            )

            self.sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )
            self.sock.settimeout(3.0)
            self.sock.connect(
                (self.rpc_host, self.rpc_port)
            )

            self.rpc = JSONRpc(self.sock)
            self.server = self.rpc.get_peer_proxy()

            self.get_logger().info("RPC connected.")

        except Exception as e:
            self.get_logger().error(f"RPC connect failed: {e}")
            self.server = None

    def close_rpc(self):
        try:
            if self.rpc is not None:
                self.rpc.close()
        except Exception:
            pass

        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass

        self.rpc = None
        self.sock = None
        self.server = None

    # ============================================================
    # 目标回调
    # ============================================================

    def target_pose_callback(self, msg):
        self.latest_target_msg = msg
        self.latest_target_time = time.time()

        # 用 frame + 坐标粗略判断是否是新目标
        target_key = (
            msg.header.frame_id,
            round(msg.pose.position.x, 3),
            round(msg.pose.position.y, 3),
            round(msg.pose.position.z, 3),
        )

        if target_key != self.current_target_key:
            self.current_target_key = target_key
            self.sent_once_for_current_target = False

        self.get_logger().info(
            f"Received semantic target: "
            f"frame={msg.header.frame_id}, "
            f"x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, "
            f"z={msg.pose.position.z:.3f}"
        )

    # ============================================================
    # 主循环
    # ============================================================

    def timer_callback(self):
        if self.latest_target_msg is None:
            return

        dt = time.time() - self.latest_target_time

        if dt > self.target_timeout:
            return

        target_in_map = self.get_target_in_map()

        if target_in_map is None:
            return

        goal = self.compute_navigation_goal(target_in_map)

        if goal is None:
            return

        self.send_goal_if_needed(goal)

    # ============================================================
    # TF
    # ============================================================

    def get_robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = yaw_from_quaternion(tf.transform.rotation)

            return x, y, yaw

        except Exception as e:
            self.get_logger().warn(f"Get robot pose in map failed: {e}")
            return None

    def get_target_in_map(self):
        if self.latest_target_msg is None:
            return None

        try:
            if self.latest_target_msg.header.frame_id == self.map_frame:
                return self.latest_target_msg

            target_in_map = self.tf_buffer.transform(
                self.latest_target_msg,
                self.map_frame,
                timeout=Duration(seconds=0.5)
            )

            return target_in_map

        except Exception as e:
            self.get_logger().warn(f"Transform target to map failed: {e}")
            return None

    # ============================================================
    # 导航目标计算
    # ============================================================

    def compute_navigation_goal(self, target_in_map):
        robot_pose = self.get_robot_pose_in_map()

        if robot_pose is None:
            return None

        robot_x, robot_y, robot_yaw = robot_pose

        obj_x = float(target_in_map.pose.position.x)
        obj_y = float(target_in_map.pose.position.y)

        dx = obj_x - robot_x
        dy = obj_y - robot_y

        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.min_target_distance:
            self.get_logger().warn(
                f"Target too close to robot, dist={dist:.3f}, skip."
            )
            return None

        ux = dx / dist
        uy = dy / dist

        # 机器人停在物体前 goal_distance 米
        goal_x = obj_x - ux * self.goal_distance
        goal_y = obj_y - uy * self.goal_distance

        # 机器人朝向物体
        goal_yaw = math.atan2(
            obj_y - goal_y,
            obj_x - goal_x
        )

        self.get_logger().info(
            f"Target map=({obj_x:.3f}, {obj_y:.3f}), "
            f"Robot map=({robot_x:.3f}, {robot_y:.3f}, {robot_yaw:.3f}), "
            f"Goal=({goal_x:.3f}, {goal_y:.3f}, {goal_yaw:.3f})"
        )

        return goal_x, goal_y, goal_yaw

    # ============================================================
    # 发送 RPC goal
    # ============================================================

    def send_goal_if_needed(self, goal):
        now = time.time()

        goal_x, goal_y, goal_yaw = goal

        if self.send_once and self.sent_once_for_current_target:
            return

        if now - self.last_goal_send_time < self.send_interval:
            return

        if self.last_sent_goal is not None:
            last_x, last_y, last_yaw = self.last_sent_goal

            goal_change_dist = distance_2d(
                goal_x,
                goal_y,
                last_x,
                last_y
            )

            goal_change_yaw = abs(
                normalize_angle(goal_yaw - last_yaw)
            )

            if (
                goal_change_dist < self.min_goal_change and
                goal_change_yaw < math.radians(10.0)
            ):
                return

        if self.server is None:
            self.connect_rpc()

            if self.server is None:
                self.get_logger().error("RPC server is not connected.")
                return

        try:
            result = self.server.add_goal(
                float(goal_x),
                float(goal_y),
                float(goal_yaw)
            )

            self.get_logger().info(
                f"RPC add_goal result: {result}, "
                f"goal=({goal_x:.3f}, {goal_y:.3f}, {goal_yaw:.3f})"
            )

            self.last_goal_send_time = now
            self.last_sent_goal = goal
            self.sent_once_for_current_target = True

        except Exception as e:
            self.get_logger().error(f"RPC add_goal failed: {e}")
            traceback.print_exc()
            self.close_rpc()

    def destroy_node(self):
        self.close_rpc()
        super().destroy_node()


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Navigation executor for semantic_yoloe_db_runtime.py. "
            "Subscribe /yoloe/target_pose and call RPC add_goal."
        )
    )

    parser.add_argument(
        "--target-pose-topic",
        type=str,
        default="/yoloe/target_pose",
        help="semantic_yoloe_db_runtime.py --mode goto 发布的目标点话题"
    )

    parser.add_argument(
        "--rpc-host",
        type=str,
        default="localhost",
        help="导航 RPC server 地址"
    )

    parser.add_argument(
        "--rpc-port",
        type=int,
        default=6000,
        help="导航 RPC server 端口"
    )

    parser.add_argument(
        "--map-frame",
        type=str,
        default="map",
        help="地图坐标系"
    )

    parser.add_argument(
        "--base-frame",
        type=str,
        default="base_link",
        help="机器人底盘坐标系"
    )

    parser.add_argument(
        "--goal-distance",
        type=float,
        default=0.8,
        help="机器人最终停在距离语义目标多少米的位置"
    )

    parser.add_argument(
        "--min-target-distance",
        type=float,
        default=0.25,
        help="目标距离机器人小于该值时不再发送导航目标"
    )

    parser.add_argument(
        "--target-timeout",
        type=float,
        default=10.0,
        help="收到目标后多少秒内有效"
    )

    parser.add_argument(
        "--send-interval",
        type=float,
        default=2.0,
        help="重复发送导航目标的最小间隔"
    )

    parser.add_argument(
        "--min-goal-change",
        type=float,
        default=0.2,
        help="新旧导航目标小于该距离时不重复发送"
    )

    parser.add_argument(
        "--tick-period",
        type=float,
        default=0.2,
        help="检查目标并发送导航的周期"
    )

    parser.add_argument(
        "--send-once",
        action="store_true",
        help="对同一个语义目标只发送一次 add_goal"
    )

    return parser


def main():
    parser = build_arg_parser()

    # parse_known_args 是为了避免和 ROS 参数冲突
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = SemanticDatabaseNavExecutor(args)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
