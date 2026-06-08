#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose, ComputePathToPose

import threading
import queue
import socket
from bsonrpc import JSONRpc, request, service_class
import time

import tf_transformations


RESULT_CODE_MAP = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "RUNNING",
    3: "CANCELING",
    4: "SUCCESS",
    5: "CANCELED",
    6: "FAILED",
}


@service_class
class RPCServerServices:
    def __init__(self, node):
        self.node = node

    @request
    def add_goal(self, x: float, y: float, yaw: float):
        self.node.add_goal(float(x), float(y), float(yaw))
        return {
            "ok": True,
            "message": f"Goal added: ({x:.2f}, {y:.2f}, {yaw:.2f})",
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }

    @request
    def check_goal(self, x: float, y: float, yaw: float):
        return self.node.check_goal(float(x), float(y), float(yaw))

    @request
    def cancel_all(self):
        self.node.cancel_all()
        return {"ok": True, "message": "All goals canceled"}

    @request
    def cancel_one(self):
        self.node.cancel_one()
        return {"ok": True, "message": "One goal canceled"}

    @request
    def get_queue_size(self):
        return self.node.get_queue_size()

    @request
    def get_state(self):
        return self.node.get_state()


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_sdk")

        self.declare_parameter("rpc_host", "127.0.0.1")
        self.declare_parameter("rpc_port", 6000)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("check_goal_timeout_sec", 3.0)
        self.declare_parameter("planner_id", "")

        self.rpc_host = self.get_parameter("rpc_host").value
        self.rpc_port = int(self.get_parameter("rpc_port").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.check_goal_timeout_sec = float(
            self.get_parameter("check_goal_timeout_sec").value
        )
        self.planner_id = self.get_parameter("planner_id").value

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.plan_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
        )

        self.state = "IDLE"
        self.last_goal = None
        self.last_check_result = None

        self.goal_queue = queue.Queue()
        self.running = True

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 NavigateToPose action server not available")
        else:
            self.get_logger().info("Nav2 NavigateToPose action server ready")

        if not self.plan_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 ComputePathToPose action server not available")
        else:
            self.get_logger().info("Nav2 ComputePathToPose action server ready")

        self.worker_thread = threading.Thread(target=self.process_queue, daemon=True)
        self.worker_thread.start()

        threading.Thread(target=self.start_rpc_server, daemon=True).start()

    def make_pose_stamped(self, x, y, yaw):
        pose = ComputePathToPose.Goal().goal
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        return pose

    def add_goal(self, x, y, yaw):
        self.goal_queue.put((float(x), float(y), float(yaw)))
        self.last_goal = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "frame_id": self.frame_id,
        }
        self.get_logger().info(f"Add goal: ({x:.2f}, {y:.2f}, {yaw:.2f})")

    def cancel_all(self):
        self.state = "CANCELED"
        with self.goal_queue.mutex:
            self.goal_queue.queue.clear()
        self.get_logger().warn("All goals canceled")

    def cancel_one(self):
        self.state = "PAUSED"
        self.get_logger().warn("One goal canceled")

    def get_queue_size(self):
        return int(self.goal_queue.qsize())

    def get_state(self):
        return {
            "state": self.state,
            "queue_size": self.get_queue_size(),
            "last_goal": self.last_goal,
            "last_check_result": self.last_check_result,
            "frame_id": self.frame_id,
        }

    def process_queue(self):
        while self.running:
            if self.state == "RUNNING":
                time.sleep(0.1)
                continue

            try:
                goal = self.goal_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            x, y, yaw = goal
            self.get_logger().info(f"Start goal ({x:.2f}, {y:.2f}, {yaw:.2f})")
            self.navigate_to(x, y, yaw)

    def navigate_to(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(x, y, yaw)

        self.state = "RUNNING"

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.state = "FAILED"
            self.get_logger().error(f"Goal response failed: {e}")
            return

        if not goal_handle.accepted:
            self.state = "FAILED"
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            wrapped = future.result()
            status = int(getattr(wrapped, "status", 0))
            self.state = RESULT_CODE_MAP.get(status, f"STATUS_{status}")
        except Exception as e:
            self.state = "FAILED"
            self.get_logger().error(f"Navigation result failed: {e}")
            return

        self.get_logger().info(f"Navigation finished, state={self.state}")

    def check_goal(self, x, y, yaw):
        """
        Check whether Nav2 can make a path to this candidate goal.
        This does not execute navigation.
        """

        if not self.plan_client.server_is_ready():
            result = {
                "ok": False,
                "reachable": False,
                "reason": "ComputePathToPose action server not ready",
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
            }
            self.last_check_result = result
            return result

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.make_pose_stamped(x, y, yaw)
        goal_msg.use_start = False
        goal_msg.planner_id = str(self.planner_id)

        done_event = threading.Event()
        data = {
            "accepted": False,
            "status": None,
            "path_len": 0,
            "error": None,
        }

        def on_goal_response(future):
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    data["accepted"] = False
                    done_event.set()
                    return

                data["accepted"] = True
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(on_result)

            except Exception as e:
                data["error"] = str(e)
                done_event.set()

        def on_result(future):
            try:
                wrapped = future.result()
                data["status"] = int(getattr(wrapped, "status", 0))

                result = wrapped.result
                path = getattr(result, "path", None)

                if path is not None:
                    data["path_len"] = len(path.poses)
                else:
                    data["path_len"] = 0

                error_code = getattr(result, "error_code", 0)
                error_msg = getattr(result, "error_msg", "")
                data["error_code"] = int(error_code)
                data["error_msg"] = str(error_msg)

            except Exception as e:
                data["error"] = str(e)

            done_event.set()

        send_future = self.plan_client.send_goal_async(goal_msg)
        send_future.add_done_callback(on_goal_response)

        ok_wait = done_event.wait(timeout=self.check_goal_timeout_sec)

        if not ok_wait:
            result = {
                "ok": False,
                "reachable": False,
                "reason": "check_goal timeout",
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
            }
            self.last_check_result = result
            return result

        if data["error"] is not None:
            result = {
                "ok": False,
                "reachable": False,
                "reason": data["error"],
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
            }
            self.last_check_result = result
            return result

        reachable = bool(data["accepted"] and data["path_len"] > 1)

        result = {
            "ok": True,
            "reachable": reachable,
            "accepted": bool(data["accepted"]),
            "status": data["status"],
            "path_len": int(data["path_len"]),
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }

        if "error_code" in data:
            result["error_code"] = data["error_code"]
            result["error_msg"] = data["error_msg"]

        self.last_check_result = result

        self.get_logger().info(
            f"check_goal ({x:.2f}, {y:.2f}, {yaw:.2f}) "
            f"reachable={reachable}, path_len={data['path_len']}"
        )

        return result

    def start_rpc_server(self):
        ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ss.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ss.bind((self.rpc_host, self.rpc_port))
        ss.listen(10)

        self.get_logger().info(f"RPC server listening on {self.rpc_host}:{self.rpc_port}")

        while self.running and rclpy.ok():
            try:
                client_sock, addr = ss.accept()
                self.get_logger().info(f"Client connected from {addr}")
                JSONRpc(client_sock, RPCServerServices(self))
            except Exception as e:
                self.get_logger().error(f"RPC server error: {e}")
                time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.worker_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()