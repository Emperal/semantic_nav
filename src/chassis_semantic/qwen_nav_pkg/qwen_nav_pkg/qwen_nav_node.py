import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import base64


class QwenNavNode(Node):

    def __init__(self):
        super().__init__('qwen_nav_node')

        self.bridge = CvBridge()

        # 订阅
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10) # frame_id: camera_color_optical_frame
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10) # frame_id: camera_depth_optical_frame

        # 发布
        self.pub = self.create_publisher(PointStamped, '/target_point', 10)

        self.rgb = None
        self.depth = None

        # Qwen API
        self.url = "http://127.0.0.1:8000/v1/chat/completions"
        self.model = "Qwen/Qwen3.5-35B-A3B"

        # 相机内参（⚠️必须换成真实值）
        self.fx = 611.99
        self.fy = 611.75
        self.cx = 642.27
        self.cy = 397.09

        # 定时器（避免每帧请求Qwen）
        self.timer = self.create_timer(2.0, self.process)

        self.get_logger().info("Qwen Nav Node started.")

    # =====================
    # 回调
    # =====================
    def rgb_callback(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # =====================
    # Qwen调用
    # =====================
    def call_qwen(self, image):

        _, buffer = cv2.imencode('.jpg', image)
        img_base64 = base64.b64encode(buffer).decode()

        prompt = "马克杯在图像中的位置？只回答：左、中、右、上、下"

        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{img_base64}"
                            }
                        }
                    ]
                }
            ],
            "max_tokens": 10
        }

        try:
            res = requests.post(self.url, json=payload, timeout=10)
            text = res.json()["choices"][0]["message"]["content"]
            return text.strip()
        except Exception as e:
            self.get_logger().error(f"Qwen调用失败: {e}")
            return None

    # =====================
    # 文本 → 点
    # =====================
    def text_to_point(self, text, w, h):

        if "左" in text:
            return int(w * 0.25), int(h / 2)
        elif "右" in text:
            return int(w * 0.75), int(h / 2)
        elif "上" in text:
            return int(w / 2), int(h * 0.25)
        elif "下" in text:
            return int(w / 2), int(h * 0.75)
        else:
            return int(w / 2), int(h / 2)

    # =====================
    # fake SAM
    # =====================
    def fake_sam(self, u, v, h, w):
        mask = np.zeros((h, w), dtype=np.uint8)
        u1 = max(0, u - 30)
        u2 = min(w, u + 30)
        v1 = max(0, v - 30)
        v2 = min(h, v + 30)
        mask[v1:v2, u1:u2] = 1
        return mask

    # =====================
    # 3D计算
    # =====================
    def compute_3d(self, u, v):

        z = float(self.depth[v, u])

        if z == 0 or np.isnan(z):
            return None

        X = (u - self.cx) * z / self.fx
        Y = (v - self.cy) * z / self.fy

        return np.array([X, Y, z])

    # =====================
    # 主逻辑
    # =====================
    def process(self):

        if self.rgb is None or self.depth is None:
            return

        h, w = self.rgb.shape[:2]

        text = self.call_qwen(self.rgb)

        if text is None:
            return

        self.get_logger().info(f"Qwen: {text}")

        # u, v = self.text_to_point(text, w, h)

        print("height: ",h)
        print("width: ",w)

        # point = self.compute_3d(u, v)

        # if point is None:
        #     self.get_logger().warn("无效深度")
        #     return

        # msg = PointStamped()
        # msg.header.frame_id = "camera_link"
        # msg.point.x = float(point[0])
        # msg.point.y = float(point[1])
        # msg.point.z = float(point[2])

        # self.pub.publish(msg)

        # self.get_logger().info(f"发布目标点: {point}")


def main(args=None):
    rclpy.init(args=args)
    node = QwenNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
