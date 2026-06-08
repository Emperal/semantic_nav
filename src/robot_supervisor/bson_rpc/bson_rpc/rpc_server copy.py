#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

import threading
import queue
import socket
from bsonrpc import JSONRpc, request, service_class
import math
import time

import tf_transformations

RESULT_CODE_MAP = {
    0: "IDLE",
    1: "ACCEPTED",
    2: "RUNNING",   # EXECUTING
    3: "CANCELING",
    4: "SUCCESS",   # SUCCEEDED
    5: "CANCELED",
    6: "FAILED"     # ABORTED
}

# ========================
# RPC 服务类
# ========================
@service_class
class RPCServerServices:
    def __init__(self, node):
        self.node = node

    @request
    def add_goal(self, x: float, y: float, yaw: float):
        self.node.add_goal(x, y, yaw)
        return f"Goal added: ({x:.2f}, {y:.2f}, {yaw:.2f})"

    @request
    def cancel_all(self):
        self.node.cancel_all()
        return "All goals canceled"

    @request
    def cancel_one(self):
        self.node.cancel_one()
        return "One goal canceled"

    @request
    def get_queue_size(self):
        return self.node.get_queue_size()

    @request
    def get_state(self):
        return self.node.state


# ========================
# ROS2 Node 类
# ========================
class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_sdk')

        # Action client
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state = 'IDLE'

        # 目标队列
        self.goal_queue = queue.Queue()
        self.running = True

        # 等待 action server
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
        else:
            self.get_logger().info("Nav2 action server ready")

        # 启动队列线程
        self.worker_thread = threading.Thread(target=self.process_queue)
        self.worker_thread.start()

        # 启动 RPC Server
        threading.Thread(target=self.start_rpc_server, daemon=True).start()

    # -----------------------
    # RPC 接口操作
    # -----------------------
    def add_goal(self, x, y, yaw):
        self.goal_queue.put((x, y, yaw))
        self.get_logger().info(f"Add goal: ({x:.2f}, {y:.2f}, {yaw:.2f})")

    def cancel_all(self):
        self.state = 'CANCELED'
        with self.goal_queue.mutex:
            self.goal_queue.queue.clear()
        self.get_logger().warn("All goals canceled")

    def cancel_one(self):
        self.state = 'PAUSED'
        self.get_logger().warn("One goal canceled")

    def get_queue_size(self):
        return self.goal_queue.qsize()

    # -----------------------
    # 队列线程
    # -----------------------
    def process_queue(self):
        while self.running:
            if self.state == 'RUNNING':
                time.sleep(0.1)
                continue

            try:
                goal = self.goal_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            x, y, yaw = goal
            self.get_logger().info(f"Start goal ({x:.2f}, {y:.2f}, {yaw:.2f})")
            self.navigate_to(x, y, yaw)

    # -----------------------
    # 发送导航目标
    # -----------------------
    def navigate_to(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # 发送 goal
        self.state = 'RUNNING'
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # -----------------------------
    # Goal Response 回调
    # -----------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = "FAILED"
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        # 获取最终结果
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    # -----------------------------
    # Feedback 回调
    # -----------------------------
    # def feedback_callback(self, goal_handle, feedback):
    #     self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f}")

    # -----------------------------
    # Result 回调
    # -----------------------------
    def result_callback(self, future):
        result = future.result().result  # WrappedResult
        code = getattr(result, 'status', 0)
        self.state = RESULT_CODE_MAP.get(code, "RUNNING")
        self.get_logger().info(f"Navigation finished, state={self.state}")
    # -----------------------
    # RPC Server
    # -----------------------
    def start_rpc_server(self):
        host = '127.0.0.1'
        port = 6000
        ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ss.bind((host, port))
        ss.listen(10)
        self.get_logger().info(f"RPC server listening on {host}:{port}")

        while True:
            client_sock, addr = ss.accept()
            self.get_logger().info(f"Client connected from {addr}")
            JSONRpc(client_sock, RPCServerServices(self))


# -----------------------
# main
# -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.worker_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
