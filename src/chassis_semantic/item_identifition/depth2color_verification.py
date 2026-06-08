#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

import cv2
import numpy as np


class RGBDAlignmentCheck(Node):
    def __init__(self):
        super().__init__('rgbd_alignment_check')

        self.bridge = CvBridge()

        self.color_topic = '/camera/color/image_raw'

        # 改成你真正的 raw aligned depth topic
        # 不能用 encoding=rgba8 的可视化图
        self.depth_topic = '/camera/depth/image_raw'

        self.color_sub = Subscriber(self, Image, self.color_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.callback)

        self.latest_color = None
        self.latest_depth_m = None

        cv2.namedWindow('rgb_click_depth')
        cv2.setMouseCallback('rgb_click_depth', self.mouse_callback)

        self.get_logger().info(f'color topic: {self.color_topic}')
        self.get_logger().info(f'depth topic: {self.depth_topic}')
        self.get_logger().info('Click RGB image to read depth at the same pixel.')

    def callback(self, color_msg, depth_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding='bgr8'
            )

            if depth_msg.encoding == '16UC1':
                depth_raw = self.bridge.imgmsg_to_cv2(
                    depth_msg,
                    desired_encoding='passthrough'
                )
                depth_m = depth_raw.astype(np.float32) / 1000.0

            elif depth_msg.encoding == '32FC1':
                depth_m = self.bridge.imgmsg_to_cv2(
                    depth_msg,
                    desired_encoding='passthrough'
                ).astype(np.float32)

            else:
                self.get_logger().warn(
                    f'Unsupported depth encoding: {depth_msg.encoding}. '
                    f'Use 16UC1 or 32FC1 raw depth, not rgba8/rgb8.'
                )
                return

            if color.shape[0] != depth_m.shape[0] or color.shape[1] != depth_m.shape[1]:
                self.get_logger().warn(
                    f'Size mismatch: color={color.shape[1]}x{color.shape[0]}, '
                    f'depth={depth_m.shape[1]}x{depth_m.shape[0]}'
                )

            self.latest_color = color
            self.latest_depth_m = depth_m

            show = color.copy()
            cv2.putText(
                show,
                'Click object in RGB image, check depth in terminal',
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.imshow('rgb_click_depth', show)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(str(e))

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.latest_depth_m is None:
            return

        h, w = self.latest_depth_m.shape[:2]

        if x < 0 or x >= w or y < 0 or y >= h:
            return

        z = float(self.latest_depth_m[y, x])

        if z <= 0.0 or np.isnan(z):
            self.get_logger().info(
                f'Clicked pixel u={x}, v={y}, depth invalid'
            )
        else:
            self.get_logger().info(
                f'Clicked pixel u={x}, v={y}, depth={z:.3f} m'
            )


def main():
    rclpy.init()
    node = RGBDAlignmentCheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()