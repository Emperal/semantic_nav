#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import yaml
import rclpy
import numpy as np

from collections import deque

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer


class OrbbecTumExporter(Node):
    """
    Export Orbbec aligned RGB-D stream to TUM RGB-D format for pySLAM.

    Output:
        dataset_dir/
        ├── rgb/
        ├── depth/
        ├── rgb.txt
        ├── depth.txt
        ├── associations.txt
        ├── groundtruth.txt
        ├── gt.freiburg
        └── camera.yaml

    Notes:
        - gt.freiburg and groundtruth.txt use TUM trajectory format:
          timestamp tx ty tz qx qy qz qw
        - If groundtruth_source is odom, this is NOT real groundtruth.
          It is odometry trajectory / pseudo groundtruth.
    """

    def __init__(self):
        super().__init__("orbbec_tum_exporter")

        # ----------------------------
        # ROS parameters
        # ----------------------------
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")

        self.declare_parameter("dataset_dir", "/home/robot2/hzf/1asemantic_nav/lib/pyslam/dataset/1")
        self.declare_parameter("save_every_n", 1)
        self.declare_parameter("max_frames", 0)

        # Orbbec depth commonly uses uint16 millimeter depth.
        self.declare_parameter("depth_scale_to_save", 1.0)

        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("sync_slop", 0.05)

        # ----------------------------
        # Groundtruth / trajectory parameters
        # ----------------------------
        # none: do not write gt
        # identity: write fixed pose for every saved RGB-D frame
        # odom: use nearest Odometry pose
        self.declare_parameter("groundtruth_source", "odom")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_buffer_size", 2000)
        self.declare_parameter("odom_max_time_diff", 0.10)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        self.dataset_dir = self.get_parameter("dataset_dir").value
        self.save_every_n = int(self.get_parameter("save_every_n").value)
        self.max_frames = int(self.get_parameter("max_frames").value)
        self.depth_scale_to_save = float(self.get_parameter("depth_scale_to_save").value)

        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.sync_slop = float(self.get_parameter("sync_slop").value)

        self.groundtruth_source = str(
            self.get_parameter("groundtruth_source").value
        ).lower()
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_buffer_size = int(self.get_parameter("odom_buffer_size").value)
        self.odom_max_time_diff = float(self.get_parameter("odom_max_time_diff").value)

        if self.groundtruth_source not in ["none", "identity", "odom"]:
            raise ValueError(
                "groundtruth_source must be one of: none, identity, odom"
            )

        self.bridge = CvBridge()

        self.rgb_dir = os.path.join(self.dataset_dir, "rgb")
        self.depth_dir = os.path.join(self.dataset_dir, "depth")

        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        self.rgb_txt_path = os.path.join(self.dataset_dir, "rgb.txt")
        self.depth_txt_path = os.path.join(self.dataset_dir, "depth.txt")
        self.association_txt_path = os.path.join(self.dataset_dir, "associations.txt")
        self.camera_yaml_path = os.path.join(self.dataset_dir, "camera.yaml")

        # pySLAM TUM reader is currently looking for gt.freiburg.
        self.gt_freiburg_path = os.path.join(self.dataset_dir, "gt.freiburg")
        self.groundtruth_txt_path = os.path.join(self.dataset_dir, "groundtruth.txt")

        self.rgb_txt = open(self.rgb_txt_path, "a", buffering=1)
        self.depth_txt = open(self.depth_txt_path, "a", buffering=1)
        self.association_txt = open(self.association_txt_path, "a", buffering=1)

        self.gt_freiburg_txt = None
        self.groundtruth_txt = None

        if self.groundtruth_source != "none":
            self.gt_freiburg_txt = open(self.gt_freiburg_path, "a", buffering=1)
            self.groundtruth_txt = open(self.groundtruth_txt_path, "a", buffering=1)

            self._write_groundtruth_header_if_empty(self.gt_freiburg_path, self.gt_freiburg_txt)
            self._write_groundtruth_header_if_empty(self.groundtruth_txt_path, self.groundtruth_txt)

        self.frame_count = 0
        self.saved_count = 0
        self.got_camera_info = False

        self.odom_buffer = deque(maxlen=self.odom_buffer_size)

        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Optional odometry subscriber
        if self.groundtruth_source == "odom":
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                100
            )
        else:
            self.odom_sub = None

        # Synchronized RGB + depth subscribers
        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        self.sync.registerCallback(self.rgbd_callback)

        self.get_logger().info("Orbbec TUM exporter started.")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"CameraInfo topic: {self.camera_info_topic}")
        self.get_logger().info(f"Dataset dir: {self.dataset_dir}")
        self.get_logger().info(f"groundtruth_source: {self.groundtruth_source}")

        if self.groundtruth_source == "odom":
            self.get_logger().info(f"Odometry topic: {self.odom_topic}")
            self.get_logger().warn(
                "Using /odom as pseudo groundtruth. "
                "This is useful for satisfying file format requirements, "
                "but it is not true groundtruth for SLAM evaluation."
            )

        if self.groundtruth_source == "identity":
            self.get_logger().warn(
                "Using identity poses as fake groundtruth. "
                "Do not use this for trajectory accuracy evaluation."
            )

    @staticmethod
    def _write_groundtruth_header_if_empty(path, file_obj):
        try:
            if os.path.getsize(path) == 0:
                file_obj.write("# timestamp tx ty tz qx qy qz qw\n")
        except FileNotFoundError:
            file_obj.write("# timestamp tx ty tz qx qy qz qw\n")

    def camera_info_callback(self, msg: CameraInfo):
        if self.got_camera_info:
            return

        k = msg.k
        d = list(msg.d)

        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        camera_data = {
            "Camera": {
                "name": "orbbec_rgbd",
                "width": int(msg.width),
                "height": int(msg.height),
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy,
                "distortion_model": msg.distortion_model,
                "distortion_coefficients": d,
                "frame_id": msg.header.frame_id,
            },
            "Depth": {
                "registered_to_rgb": True,
                "input_depth_topic": self.depth_topic,
                "saved_format": "uint16_png",
                "depth_scale_to_save": self.depth_scale_to_save,
                "note": (
                    "If saved depth is uint16 millimeters, set pySLAM/ORB-SLAM RGB-D "
                    "depth factor to 1000.0. If converted to TUM factor format, use 5000.0."
                )
            },
            "GroundTruth": {
                "source": self.groundtruth_source,
                "odom_topic": self.odom_topic if self.groundtruth_source == "odom" else "",
                "format": "TUM timestamp tx ty tz qx qy qz qw",
                "gt_freiburg": self.gt_freiburg_path if self.groundtruth_source != "none" else "",
                "groundtruth_txt": self.groundtruth_txt_path if self.groundtruth_source != "none" else "",
                "note": (
                    "Odometry-generated trajectory is pseudo groundtruth, not true groundtruth."
                )
            }
        }

        with open(self.camera_yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(camera_data, f, allow_unicode=True, sort_keys=False)

        self.got_camera_info = True

        self.get_logger().info("Camera info saved:")
        self.get_logger().info(f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}")
        self.get_logger().info(f"camera.yaml: {self.camera_yaml_path}")

    @staticmethod
    def stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def stamp_to_name(stamp):
        return f"{stamp.sec}.{int(stamp.nanosec / 1000):06d}"

    def odom_callback(self, msg: Odometry):
        t = self.stamp_to_sec(msg.header.stamp)

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.odom_buffer.append(
            (
                t,
                float(p.x), float(p.y), float(p.z),
                float(q.x), float(q.y), float(q.z), float(q.w)
            )
        )

    def get_nearest_odom_pose(self, target_time):
        if not self.odom_buffer:
            return None

        nearest = min(self.odom_buffer, key=lambda item: abs(item[0] - target_time))
        dt = abs(nearest[0] - target_time)

        if dt > self.odom_max_time_diff:
            return None

        return nearest

    def write_groundtruth_pose(self, timestamp):
        if self.groundtruth_source == "none":
            return

        if self.groundtruth_source == "identity":
            # timestamp tx ty tz qx qy qz qw
            line = f"{timestamp:.6f} 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000\n"

        elif self.groundtruth_source == "odom":
            pose = self.get_nearest_odom_pose(timestamp)

            if pose is None:
                self.get_logger().warn(
                    f"No odom pose close to RGB timestamp {timestamp:.6f}; "
                    f"writing identity pose instead."
                )
                line = f"{timestamp:.6f} 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000\n"
            else:
                _, tx, ty, tz, qx, qy, qz, qw = pose
                line = (
                    f"{timestamp:.6f} "
                    f"{tx:.6f} {ty:.6f} {tz:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
                )

        else:
            return

        if self.gt_freiburg_txt is not None:
            self.gt_freiburg_txt.write(line)

        if self.groundtruth_txt is not None:
            self.groundtruth_txt.write(line)

    def convert_rgb(self, rgb_msg: Image):
        """
        Convert ROS Image to BGR image for cv2.imwrite.
        Handles common encodings from Orbbec.
        """
        enc = rgb_msg.encoding.lower()

        if enc in ["rgb8", "bgr8"]:
            img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            return img

        if enc in ["rgba8", "bgra8"]:
            rgba = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgba8")
            bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
            return bgr

        # fallback
        img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="passthrough")

        if len(img.shape) == 2:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        if img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

        if img.shape[2] == 3:
            return img

        raise RuntimeError(f"Unsupported RGB image shape: {img.shape}, encoding: {rgb_msg.encoding}")

    def convert_depth(self, depth_msg: Image):
        """
        Convert ROS depth image to uint16 PNG.

        Supported:
            16UC1: usually millimeters
            32FC1: usually meters, converted to millimeters
        """
        enc = depth_msg.encoding.lower()
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        if enc in ["16uc1", "mono16"]:
            depth_u16 = depth.astype(np.uint16)

        elif enc == "32fc1":
            # meters -> millimeters
            depth_u16 = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth_u16 = np.clip(depth_u16 * 1000.0, 0, 65535).astype(np.uint16)

        else:
            raise RuntimeError(
                f"Depth topic encoding is {depth_msg.encoding}. "
                f"Expected 16UC1 or 32FC1. "
                f"Please check whether you are using the real aligned depth topic."
            )

        if self.depth_scale_to_save != 1.0:
            depth_u16 = np.clip(
                depth_u16.astype(np.float32) * self.depth_scale_to_save,
                0,
                65535
            ).astype(np.uint16)

        return depth_u16

    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        self.frame_count += 1

        if self.save_every_n > 1:
            if self.frame_count % self.save_every_n != 0:
                return

        if self.max_frames > 0 and self.saved_count >= self.max_frames:
            self.get_logger().info("Reached max_frames, shutting down exporter.")
            rclpy.shutdown()
            return

        try:
            rgb_img = self.convert_rgb(rgb_msg)
            depth_img = self.convert_depth(depth_msg)

            # Use RGB timestamp as main timestamp.
            rgb_time = self.stamp_to_sec(rgb_msg.header.stamp)
            depth_time = self.stamp_to_sec(depth_msg.header.stamp)

            rgb_name = self.stamp_to_name(rgb_msg.header.stamp)
            depth_name = self.stamp_to_name(depth_msg.header.stamp)

            rgb_rel_path = f"rgb/{rgb_name}.png"
            depth_rel_path = f"depth/{depth_name}.png"

            rgb_abs_path = os.path.join(self.dataset_dir, rgb_rel_path)
            depth_abs_path = os.path.join(self.dataset_dir, depth_rel_path)

            ok_rgb = cv2.imwrite(rgb_abs_path, rgb_img)
            ok_depth = cv2.imwrite(depth_abs_path, depth_img)

            if not ok_rgb:
                raise RuntimeError(f"Failed to write RGB image: {rgb_abs_path}")

            if not ok_depth:
                raise RuntimeError(f"Failed to write depth image: {depth_abs_path}")

            self.rgb_txt.write(f"{rgb_time:.6f} {rgb_rel_path}\n")
            self.depth_txt.write(f"{depth_time:.6f} {depth_rel_path}\n")

            # TUM association format:
            # rgb_timestamp rgb_file depth_timestamp depth_file
            self.association_txt.write(
                f"{rgb_time:.6f} {rgb_rel_path} {depth_time:.6f} {depth_rel_path}\n"
            )

            # Write groundtruth / pseudo groundtruth.
            self.write_groundtruth_pose(rgb_time)

            self.saved_count += 1

            if self.saved_count % 30 == 0:
                dt_ms = abs(rgb_time - depth_time) * 1000.0
                self.get_logger().info(
                    f"Saved {self.saved_count} RGB-D frames. "
                    f"RGB-depth dt={dt_ms:.2f} ms"
                )

        except Exception as e:
            self.get_logger().error(f"Failed to save RGB-D frame: {e}")

    def destroy_node(self):
        try:
            self.rgb_txt.close()
            self.depth_txt.close()
            self.association_txt.close()

            if self.gt_freiburg_txt is not None:
                self.gt_freiburg_txt.close()

            if self.groundtruth_txt is not None:
                self.groundtruth_txt.close()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecTumExporter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()