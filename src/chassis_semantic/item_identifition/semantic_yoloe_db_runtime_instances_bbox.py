#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
semantic_yoloe_db_runtime_instances_bbox.py

版本特点：
1. 支持同 label 多实例：
   chair_1、chair_2、bottle_1 ...

2. 同一实例判断：
   label 相同，并且 map 坐标距离 < same_instance_distance，
   则认为是同一个物体实例。

3. 同一实例保留逻辑：
   不再简单用新观测更新代表位置。
   而是比较 bbox 面积：
      新 bbox_area > 数据库中该实例 best_bbox_area
          -> 用新观测替换该实例的代表 bbox、代表 camera 坐标、代表 map 坐标、代表图片
      否则
          -> 只增加观测记录，不改变该实例的代表位置

4. 所有观测都会保存到 semantic_observations 表；
   semantic_objects 表只保存每个实例的“最佳代表观测”。

5. 后续 goto 导航时使用 semantic_objects 表里的代表 map 坐标：
      rep_map_x, rep_map_y, rep_map_z

运行示例：

建库：
python3 semantic_yoloe_db_runtime_instances_bbox.py \
  --mode build \
  --weights-path /home/robot2/hzf/2D_robot_ros2_final/src/bringup/weights/yoloe-26s-seg-pf.pt \
  --sample-period 1.0 \
  --same-instance-distance 0.6 \
  --show-window

查询：
python3 semantic_yoloe_db_runtime_instances_bbox.py \
  --mode query \
  --query "椅子"

导航到最近的椅子：
python3 semantic_yoloe_db_runtime_instances_bbox.py \
  --mode goto \
  --query "椅子" \
  --select-policy nearest

导航到指定实例：
python3 semantic_yoloe_db_runtime_instances_bbox.py \
  --mode goto \
  --query "chair_2"
"""

import os
import cv2
import time
import json
import math
import sqlite3
import argparse
import traceback
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped

from cv_bridge import CvBridge
from ultralytics import YOLO

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


CN_ALIAS = {
    "椅子": "chair",
    "座椅": "chair",
    "凳子": "chair",
    "水瓶": "bottle",
    "瓶子": "bottle",
    "杯子": "cup",
    "水杯": "cup",
    "桌子": "table",
    "桌": "table",
    "人": "person",
    "行人": "person",
    "箱子": "box",
    "盒子": "box",
    "门": "door",
    "垃圾桶": "trash can",
    "背包": "backpack",
    "电脑": "computer",
    "显示器": "monitor",
    "键盘": "keyboard",
    "鼠标": "mouse",
    "手机": "phone"
}


def normalize_label(label):
    return str(label).strip().lower().replace("_", " ")


def parse_classes(text):
    if text is None:
        return []

    result = []
    for item in text.split(","):
        item = normalize_label(item)
        if item:
            result.append(item)
    return result


def semantic_query_to_label(query_text):
    q = str(query_text).strip().lower()

    for cn, en in CN_ALIAS.items():
        if cn in query_text:
            return en

    return q


def bbox_area(bbox):
    x1, y1, x2, y2 = bbox
    w = max(0.0, float(x2) - float(x1))
    h = max(0.0, float(y2) - float(y1))
    return w * h


def dist2d(x1, y1, x2, y2):
    if x1 is None or y1 is None or x2 is None or y2 is None:
        return None
    dx = float(x1) - float(x2)
    dy = float(y1) - float(y2)
    return math.sqrt(dx * dx + dy * dy)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SemanticYoloeInstanceDatabaseBuilder(Node):
    def __init__(self, args):
        super().__init__("semantic_yoloe_instance_bbox_database_builder")

        self.args = args

        self.image_topic = args.image_topic
        self.depth_topic = args.depth_topic
        self.color_info_topic = args.color_info_topic
        self.depth_info_topic = args.depth_info_topic

        self.map_frame = args.map_frame
        self.base_frame = args.base_frame

        self.weights_path = args.weights_path
        self.class_names = parse_classes(args.classes)

        self.min_confidence = float(args.min_confidence)
        self.depth_scale = float(args.depth_scale)
        self.min_depth = float(args.min_depth)
        self.max_depth = float(args.max_depth)
        self.roi_size = int(args.roi_size)

        self.sample_period = float(args.sample_period)
        self.same_instance_distance = float(args.same_instance_distance)
        self.show_window = bool(args.show_window)

        self.db_path = args.db_path
        self.image_save_dir = args.image_save_dir

        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        os.makedirs(self.image_save_dir, exist_ok=True)

        self.bridge = CvBridge()

        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self.latest_depth_header = None
        self.color_info = None
        self.depth_info = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.aligned_depth_pub = self.create_publisher(
            Image,
            "/yoloe/aligned_depth_to_color",
            10
        )

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.color_info_topic, self.color_info_callback, 10)
        self.create_subscription(CameraInfo, self.depth_info_topic, self.depth_info_callback, 10)

        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.init_database()

        self.get_logger().info(f"Loading YOLOE model: {self.weights_path}")
        self.model = YOLO(self.weights_path)
        self.get_logger().info("YOLOE loaded.")

        self.timer = self.create_timer(self.sample_period, self.sample_once)

        self.get_logger().info("Semantic YOLOE instance bbox database builder started.")
        self.get_logger().info(f"sample_period: {self.sample_period}")
        self.get_logger().info(f"same_instance_distance: {self.same_instance_distance}")
        self.get_logger().info(f"db_path: {self.db_path}")
        self.get_logger().info(f"classes: {self.class_names if self.class_names else 'all'}")

    # ============================================================
    # 数据库
    # ============================================================

    def init_database(self):
        cur = self.conn.cursor()

        cur.execute("""
        CREATE TABLE IF NOT EXISTS semantic_objects (
            id INTEGER PRIMARY KEY AUTOINCREMENT,

            label TEXT NOT NULL,
            label_norm TEXT NOT NULL,

            instance_id INTEGER NOT NULL,
            instance_name TEXT NOT NULL,

            observe_count INTEGER DEFAULT 0,
            best_confidence REAL DEFAULT 0.0,

            first_seen_time TEXT,
            last_seen_time TEXT,

            best_bbox_area REAL DEFAULT 0.0,
            best_bbox_x1 REAL,
            best_bbox_y1 REAL,
            best_bbox_x2 REAL,
            best_bbox_y2 REAL,

            rep_camera_x REAL,
            rep_camera_y REAL,
            rep_camera_z REAL,

            rep_map_x REAL,
            rep_map_y REAL,
            rep_map_z REAL,

            camera_frame TEXT,
            map_frame TEXT,
            rep_image_path TEXT
        );
        """)

        cur.execute("""
        CREATE TABLE IF NOT EXISTS semantic_observations (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            object_id INTEGER,

            label TEXT NOT NULL,
            label_norm TEXT NOT NULL,

            instance_id INTEGER,
            instance_name TEXT,

            instance_action TEXT,
            match_distance REAL,

            confidence REAL,

            time_text TEXT,
            stamp_sec INTEGER,
            stamp_nanosec INTEGER,

            bbox_area REAL,
            bbox_x1 REAL,
            bbox_y1 REAL,
            bbox_x2 REAL,
            bbox_y2 REAL,

            camera_x REAL,
            camera_y REAL,
            camera_z REAL,

            map_x REAL,
            map_y REAL,
            map_z REAL,

            camera_frame TEXT,
            map_frame TEXT,

            image_path TEXT,
            raw_json TEXT,

            FOREIGN KEY(object_id) REFERENCES semantic_objects(id)
        );
        """)

        cur.execute("""
        CREATE INDEX IF NOT EXISTS idx_semantic_objects_label
        ON semantic_objects(label_norm);
        """)

        cur.execute("""
        CREATE INDEX IF NOT EXISTS idx_semantic_objects_instance
        ON semantic_objects(instance_name);
        """)

        cur.execute("""
        CREATE INDEX IF NOT EXISTS idx_semantic_observations_label
        ON semantic_observations(label_norm);
        """)

        self.conn.commit()

    def get_next_instance_id(self, label_norm):
        cur = self.conn.cursor()
        cur.execute("""
            SELECT MAX(instance_id)
            FROM semantic_objects
            WHERE label_norm = ?
        """, (label_norm,))
        row = cur.fetchone()
        max_id = row[0] if row is not None else None
        if max_id is None:
            return 1
        return int(max_id) + 1

    def find_matching_instance(self, label_norm, map_xyz):
        """
        根据 label + map 坐标距离寻找实例。
        匹配成功后是否替换代表观测，交给 bbox_area 决定。
        """
        mx, my, mz = map_xyz

        if mx is None or my is None:
            return None

        cur = self.conn.cursor()
        cur.execute("""
            SELECT
                id,
                label,
                label_norm,
                instance_id,
                instance_name,
                observe_count,
                best_confidence,
                best_bbox_area,
                rep_map_x,
                rep_map_y,
                rep_map_z
            FROM semantic_objects
            WHERE label_norm = ?
              AND rep_map_x IS NOT NULL
              AND rep_map_y IS NOT NULL
        """, (label_norm,))

        rows = cur.fetchall()

        best_row = None
        best_dist = None

        for row in rows:
            rep_map_x = row[8]
            rep_map_y = row[9]

            d = dist2d(mx, my, rep_map_x, rep_map_y)

            if d is None:
                continue

            if best_dist is None or d < best_dist:
                best_dist = d
                best_row = row

        if best_row is not None and best_dist is not None:
            if best_dist <= self.same_instance_distance:
                return {
                    "id": best_row[0],
                    "label": best_row[1],
                    "label_norm": best_row[2],
                    "instance_id": best_row[3],
                    "instance_name": best_row[4],
                    "observe_count": best_row[5],
                    "best_confidence": best_row[6],
                    "best_bbox_area": best_row[7],
                    "rep_map_x": best_row[8],
                    "rep_map_y": best_row[9],
                    "rep_map_z": best_row[10],
                    "match_distance": best_dist
                }

        return None

    def create_new_instance(
        self,
        label,
        label_norm,
        confidence,
        bbox,
        bbox_area_value,
        camera_xyz,
        map_xyz,
        camera_frame,
        map_frame,
        image_path,
        now_text
    ):
        instance_id = self.get_next_instance_id(label_norm)
        instance_name = f"{label_norm.replace(' ', '_')}_{instance_id}"

        x1, y1, x2, y2 = bbox
        cx, cy, cz = camera_xyz
        mx, my, mz = map_xyz

        cur = self.conn.cursor()
        cur.execute("""
            INSERT INTO semantic_objects (
                label,
                label_norm,

                instance_id,
                instance_name,

                observe_count,
                best_confidence,

                first_seen_time,
                last_seen_time,

                best_bbox_area,
                best_bbox_x1,
                best_bbox_y1,
                best_bbox_x2,
                best_bbox_y2,

                rep_camera_x,
                rep_camera_y,
                rep_camera_z,

                rep_map_x,
                rep_map_y,
                rep_map_z,

                camera_frame,
                map_frame,
                rep_image_path
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            label,
            label_norm,

            instance_id,
            instance_name,

            1,
            float(confidence),

            now_text,
            now_text,

            float(bbox_area_value),
            float(x1),
            float(y1),
            float(x2),
            float(y2),

            float(cx),
            float(cy),
            float(cz),

            None if mx is None else float(mx),
            None if my is None else float(my),
            None if mz is None else float(mz),

            camera_frame,
            map_frame,
            image_path
        ))

        object_id = cur.lastrowid

        return object_id, instance_id, instance_name, "new", None

    def update_instance_by_bbox_rule(
        self,
        obj,
        confidence,
        bbox,
        bbox_area_value,
        camera_xyz,
        map_xyz,
        camera_frame,
        map_frame,
        image_path,
        now_text
    ):
        """
        同一实例时：
        1. observe_count 一定 +1
        2. best_confidence 更新为最大值
        3. 只有当新 bbox_area 更大，才替换代表 bbox、代表位置、代表图片
        """
        object_id = obj["id"]
        observe_count = int(obj["observe_count"])
        best_confidence = float(obj["best_confidence"])
        old_best_area = float(obj["best_bbox_area"] or 0.0)

        new_count = observe_count + 1
        new_best_confidence = max(best_confidence, float(confidence))

        x1, y1, x2, y2 = bbox
        cx, cy, cz = camera_xyz
        mx, my, mz = map_xyz

        replace_representative = float(bbox_area_value) > old_best_area

        cur = self.conn.cursor()

        if replace_representative:
            cur.execute("""
                UPDATE semantic_objects
                SET
                    observe_count = ?,
                    best_confidence = ?,
                    last_seen_time = ?,

                    best_bbox_area = ?,
                    best_bbox_x1 = ?,
                    best_bbox_y1 = ?,
                    best_bbox_x2 = ?,
                    best_bbox_y2 = ?,

                    rep_camera_x = ?,
                    rep_camera_y = ?,
                    rep_camera_z = ?,

                    rep_map_x = ?,
                    rep_map_y = ?,
                    rep_map_z = ?,

                    camera_frame = ?,
                    map_frame = ?,
                    rep_image_path = ?
                WHERE id = ?
            """, (
                new_count,
                new_best_confidence,
                now_text,

                float(bbox_area_value),
                float(x1),
                float(y1),
                float(x2),
                float(y2),

                float(cx),
                float(cy),
                float(cz),

                None if mx is None else float(mx),
                None if my is None else float(my),
                None if mz is None else float(mz),

                camera_frame,
                map_frame,
                image_path,
                object_id
            ))
            action = "update_replace_by_larger_bbox"
        else:
            cur.execute("""
                UPDATE semantic_objects
                SET
                    observe_count = ?,
                    best_confidence = ?,
                    last_seen_time = ?
                WHERE id = ?
            """, (
                new_count,
                new_best_confidence,
                now_text,
                object_id
            ))
            action = "update_keep_old_bbox"

        return object_id, obj["instance_id"], obj["instance_name"], action, obj.get("match_distance", None)

    def save_detection(
        self,
        label,
        confidence,
        bbox,
        camera_xyz,
        map_xyz,
        camera_frame,
        map_frame,
        stamp,
        image_path,
        raw_dict
    ):
        label_norm = normalize_label(label)
        x1, y1, x2, y2 = bbox
        cx, cy, cz = camera_xyz
        mx, my, mz = map_xyz

        area = bbox_area(bbox)
        now_text = datetime.now().isoformat(timespec="seconds")

        matched_obj = self.find_matching_instance(label_norm, map_xyz)

        if matched_obj is None:
            object_id, instance_id, instance_name, action, match_distance = self.create_new_instance(
                label=label,
                label_norm=label_norm,
                confidence=confidence,
                bbox=bbox,
                bbox_area_value=area,
                camera_xyz=camera_xyz,
                map_xyz=map_xyz,
                camera_frame=camera_frame,
                map_frame=map_frame,
                image_path=image_path,
                now_text=now_text
            )
        else:
            object_id, instance_id, instance_name, action, match_distance = self.update_instance_by_bbox_rule(
                obj=matched_obj,
                confidence=confidence,
                bbox=bbox,
                bbox_area_value=area,
                camera_xyz=camera_xyz,
                map_xyz=map_xyz,
                camera_frame=camera_frame,
                map_frame=map_frame,
                image_path=image_path,
                now_text=now_text
            )

        raw_dict["instance_id"] = instance_id
        raw_dict["instance_name"] = instance_name
        raw_dict["instance_action"] = action
        raw_dict["match_distance"] = match_distance
        raw_dict["same_instance_distance"] = self.same_instance_distance
        raw_dict["bbox_area"] = float(area)

        cur = self.conn.cursor()
        cur.execute("""
            INSERT INTO semantic_observations (
                object_id,

                label,
                label_norm,

                instance_id,
                instance_name,

                instance_action,
                match_distance,

                confidence,

                time_text,
                stamp_sec,
                stamp_nanosec,

                bbox_area,
                bbox_x1,
                bbox_y1,
                bbox_x2,
                bbox_y2,

                camera_x,
                camera_y,
                camera_z,

                map_x,
                map_y,
                map_z,

                camera_frame,
                map_frame,

                image_path,
                raw_json
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            object_id,

            label,
            label_norm,

            instance_id,
            instance_name,

            action,
            None if match_distance is None else float(match_distance),

            float(confidence),

            now_text,
            int(stamp.sec),
            int(stamp.nanosec),

            float(area),
            float(x1),
            float(y1),
            float(x2),
            float(y2),

            float(cx),
            float(cy),
            float(cz),

            None if mx is None else float(mx),
            None if my is None else float(my),
            None if mz is None else float(mz),

            camera_frame,
            map_frame,

            image_path,
            json.dumps(raw_dict, ensure_ascii=False)
        ))

        self.conn.commit()

        return instance_name, action, match_distance, area

    # ============================================================
    # ROS 回调
    # ============================================================

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_color = image.copy()
            self.latest_color_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Image convert failed: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth = np.asarray(depth)

            if depth.ndim == 3:
                depth = depth[:, :, 0]

            self.latest_depth = depth.copy()
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth convert failed: {e}")

    def color_info_callback(self, msg):
        self.color_info = msg

    def depth_info_callback(self, msg):
        self.depth_info = msg

    # ============================================================
    # 深度对齐
    # ============================================================

    @staticmethod
    def quat_to_rot(qx, qy, qz, qw):
        x, y, z, w = qx, qy, qz, qw
        return np.array([
            [1 - 2 * y * y - 2 * z * z,
             2 * x * y - 2 * z * w,
             2 * x * z + 2 * y * w],

            [2 * x * y + 2 * z * w,
             1 - 2 * x * x - 2 * z * z,
             2 * y * z - 2 * x * w],

            [2 * x * z - 2 * y * w,
             2 * y * z + 2 * x * w,
             1 - 2 * x * x - 2 * y * y],
        ], dtype=np.float64)

    def get_depth_to_color_tf(self, depth_frame, color_frame):
        if depth_frame == color_frame:
            return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)

        try:
            trans = self.tf_buffer.lookup_transform(
                color_frame,
                depth_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )

            t = trans.transform.translation
            q = trans.transform.rotation

            R = self.quat_to_rot(q.x, q.y, q.z, q.w)
            T = np.array([t.x, t.y, t.z], dtype=np.float64)
            return R, T

        except Exception as e:
            self.get_logger().warn(
                f"Cannot get TF from {depth_frame} to {color_frame}: {e}"
            )
            return None, None

    def align_depth_to_color(self, depth, depth_info, color_info, color_shape):
        h_c, w_c = color_shape[:2]
        h_d, w_d = depth.shape[:2]

        fx_d = float(depth_info.k[0])
        fy_d = float(depth_info.k[4])
        cx_d = float(depth_info.k[2])
        cy_d = float(depth_info.k[5])

        fx_c = float(color_info.k[0])
        fy_c = float(color_info.k[4])
        cx_c = float(color_info.k[2])
        cy_c = float(color_info.k[5])

        if fx_d <= 0 or fy_d <= 0 or fx_c <= 0 or fy_c <= 0:
            self.get_logger().warn("Invalid camera intrinsics.")
            return None

        depth_frame = depth_info.header.frame_id
        color_frame = color_info.header.frame_id

        R, T = self.get_depth_to_color_tf(depth_frame, color_frame)

        if R is None:
            return None

        aligned_depth = np.zeros((h_c, w_c), dtype=np.float32)

        for v_d in range(h_d):
            for u_d in range(w_d):
                z_raw = float(depth[v_d, u_d])

                if z_raw <= 0 or np.isnan(z_raw):
                    continue

                z_d = z_raw * self.depth_scale

                if z_d < self.min_depth or z_d > self.max_depth:
                    continue

                x_d = (u_d - cx_d) * z_d / fx_d
                y_d = (v_d - cy_d) * z_d / fy_d

                p_d = np.array([x_d, y_d, z_d], dtype=np.float64)
                p_c = R @ p_d + T

                x_c, y_c, z_c = p_c.tolist()

                if z_c <= 0:
                    continue

                u_c = int(round(fx_c * x_c / z_c + cx_c))
                v_c = int(round(fy_c * y_c / z_c + cy_c))

                if u_c < 0 or u_c >= w_c or v_c < 0 or v_c >= h_c:
                    continue

                old_z = aligned_depth[v_c, u_c]
                if old_z == 0.0 or z_c < old_z:
                    aligned_depth[v_c, u_c] = float(z_c)

        return aligned_depth

    # ============================================================
    # 周期采集
    # ============================================================

    def sample_once(self):
        color = self.latest_color.copy() if self.latest_color is not None else None
        color_header = self.latest_color_header

        depth = self.latest_depth.copy() if self.latest_depth is not None else None
        depth_header = self.latest_depth_header

        color_info = self.color_info
        depth_info = self.depth_info

        if color is None:
            self.get_logger().warn("No color image received yet.")
            return

        if depth is None:
            self.get_logger().warn("No depth image received yet.")
            return

        if color_info is None or depth_info is None:
            self.get_logger().warn("No camera info received yet.")
            return

        try:
            self.process_frame(
                color=color,
                color_header=color_header,
                depth=depth,
                depth_header=depth_header,
                color_info=color_info,
                depth_info=depth_info
            )
        except Exception as e:
            self.get_logger().error(f"process_frame failed: {e}")
            traceback.print_exc()

    def process_frame(self, color, color_header, depth, depth_header, color_info, depth_info):
        aligned_depth = self.align_depth_to_color(
            depth=depth,
            depth_info=depth_info,
            color_info=color_info,
            color_shape=color.shape
        )

        if aligned_depth is None:
            self.get_logger().warn("aligned_depth is None, skip this frame.")
            return

        aligned_depth_msg = self.bridge.cv2_to_imgmsg(aligned_depth, encoding="32FC1")
        aligned_depth_msg.header.stamp = depth_header.stamp
        aligned_depth_msg.header.frame_id = color_info.header.frame_id
        self.aligned_depth_pub.publish(aligned_depth_msg)

        results = self.model(color, verbose=False)
        res = results[0]
        overlay = res.plot()

        if res.boxes is None or len(res.boxes) == 0:
            self.get_logger().info("No YOLOE detections.")
            if self.show_window:
                cv2.imshow("semantic instance bbox database builder", overlay)
                cv2.waitKey(1)
            return

        boxes = res.boxes.xyxy.detach().cpu().numpy()
        confs = res.boxes.conf.detach().cpu().numpy()
        cls_ids = res.boxes.cls.detach().cpu().numpy().astype(int)

        detections = []

        for i in range(len(boxes)):
            conf = float(confs[i])

            if conf < self.min_confidence:
                continue

            cls_id = int(cls_ids[i])

            if hasattr(res, "names") and cls_id in res.names:
                label = normalize_label(res.names[cls_id])
            else:
                label = normalize_label(cls_id)

            if len(self.class_names) > 0 and label not in self.class_names:
                continue

            camera_xyz = self.compute_object_camera_position(
                bbox=boxes[i],
                aligned_depth=aligned_depth,
                color_info=color_info
            )

            if camera_xyz[0] is None:
                continue

            map_xyz = self.transform_camera_xyz_to_map(
                camera_xyz=camera_xyz,
                camera_frame=color_info.header.frame_id,
                stamp=color_header.stamp
            )

            detections.append({
                "label": label,
                "confidence": conf,
                "bbox": boxes[i],
                "camera_xyz": camera_xyz,
                "map_xyz": map_xyz,
                "cls_id": cls_id
            })

        if len(detections) == 0:
            self.get_logger().info("No valid detections with depth.")
            if self.show_window:
                cv2.imshow("semantic instance bbox database builder", overlay)
                cv2.waitKey(1)
            return

        image_path = self.save_image(overlay, color_header.stamp)

        saved_count = 0
        valid_map_count = 0

        for det in detections:
            bbox = det["bbox"]
            camera_xyz = det["camera_xyz"]
            map_xyz = det["map_xyz"]
            area = bbox_area(bbox)

            if map_xyz[0] is not None:
                valid_map_count += 1

            raw_dict = {
                "label": det["label"],
                "confidence": float(det["confidence"]),
                "bbox": [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])],
                "bbox_area": float(area),
                "camera_xyz": [float(camera_xyz[0]), float(camera_xyz[1]), float(camera_xyz[2])],
                "map_xyz": [
                    None if map_xyz[0] is None else float(map_xyz[0]),
                    None if map_xyz[1] is None else float(map_xyz[1]),
                    None if map_xyz[2] is None else float(map_xyz[2]),
                ],
                "camera_frame": color_info.header.frame_id,
                "map_frame": self.map_frame,
                "stamp": {
                    "sec": int(color_header.stamp.sec),
                    "nanosec": int(color_header.stamp.nanosec)
                }
            }

            instance_name, action, match_distance, area = self.save_detection(
                label=det["label"],
                confidence=det["confidence"],
                bbox=bbox,
                camera_xyz=camera_xyz,
                map_xyz=map_xyz,
                camera_frame=color_info.header.frame_id,
                map_frame=self.map_frame,
                stamp=color_header.stamp,
                image_path=image_path,
                raw_dict=raw_dict
            )

            self.draw_detection(
                overlay=overlay,
                instance_name=instance_name,
                action=action,
                confidence=det["confidence"],
                bbox=bbox,
                bbox_area_value=area,
                map_xyz=map_xyz
            )

            saved_count += 1

        self.get_logger().info(
            f"Saved {saved_count} observations. "
            f"Valid map positions: {valid_map_count}. "
            f"image={image_path}"
        )

        if self.show_window:
            cv2.imshow("semantic instance bbox database builder", overlay)
            cv2.waitKey(1)

    # ============================================================
    # 位置计算
    # ============================================================

    def compute_object_camera_position(self, bbox, aligned_depth, color_info):
        h, w = aligned_depth.shape[:2]

        x1, y1, x2, y2 = bbox

        x1 = max(0, int(x1))
        y1 = max(0, int(y1))
        x2 = min(w - 1, int(x2))
        y2 = min(h - 1, int(y2))

        if x2 <= x1 or y2 <= y1:
            return None, None, None

        u = int((x1 + x2) / 2.0)
        v = int((y1 + y2) / 2.0)

        roi = aligned_depth[
            max(0, v - self.roi_size):min(h, v + self.roi_size),
            max(0, u - self.roi_size):min(w, u + self.roi_size)
        ]

        valid = roi[
            np.isfinite(roi) &
            (roi > self.min_depth) &
            (roi < self.max_depth)
        ]

        if valid.size < 5:
            return None, None, None

        z = float(np.median(valid))

        fx = float(color_info.k[0])
        fy = float(color_info.k[4])
        cx = float(color_info.k[2])
        cy = float(color_info.k[5])

        if fx <= 0 or fy <= 0:
            return None, None, None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return float(x), float(y), float(z)

    def transform_camera_xyz_to_map(self, camera_xyz, camera_frame, stamp):
        x, y, z = camera_xyz

        p_cam = PointStamped()
        p_cam.header.stamp = stamp
        p_cam.header.frame_id = camera_frame
        p_cam.point.x = float(x)
        p_cam.point.y = float(y)
        p_cam.point.z = float(z)

        try:
            p_map = self.tf_buffer.transform(
                p_cam,
                self.map_frame,
                timeout=Duration(seconds=0.5)
            )

            return (
                float(p_map.point.x),
                float(p_map.point.y),
                float(p_map.point.z)
            )

        except Exception as e:
            self.get_logger().warn(
                f"Transform camera point to {self.map_frame} failed: {e}"
            )
            return None, None, None

    # ============================================================
    # 可视化
    # ============================================================

    def draw_detection(self, overlay, instance_name, action, confidence, bbox,
                       bbox_area_value, map_xyz):
        x1, y1, x2, y2 = bbox.astype(int)
        map_x, map_y, map_z = map_xyz

        if map_x is None:
            text = (
                f"{instance_name} {action} conf={confidence:.2f} "
                f"area={bbox_area_value:.0f} map=None"
            )
        else:
            text = (
                f"{instance_name} {action} conf={confidence:.2f} "
                f"area={bbox_area_value:.0f} map=({map_x:.2f},{map_y:.2f})"
            )

        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        cv2.circle(overlay, (u, v), 5, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            text,
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            2
        )

    def save_image(self, image, stamp):
        filename = f"{stamp.sec}_{stamp.nanosec}.jpg"
        path = os.path.join(self.image_save_dir, filename)
        cv2.imwrite(path, image)
        return path

    def destroy_node(self):
        if hasattr(self, "conn"):
            self.conn.close()
        cv2.destroyAllWindows()
        super().destroy_node()


# ============================================================
# 查询与发布导航目标
# ============================================================

def get_robot_pose_in_map_once(map_frame, base_frame):
    rclpy.init()
    node = rclpy.create_node("semantic_query_tf_reader")
    tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    pose = None
    start = time.time()

    while time.time() - start < 1.5:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = yaw_from_quaternion(tf.transform.rotation)
            pose = (x, y, yaw)
            break
        except Exception:
            pass

    node.destroy_node()
    rclpy.shutdown()
    return pose


def query_database(db_path, query_text, select_policy="nearest",
                   robot_pose=None, map_frame="map"):
    if not os.path.exists(db_path):
        return {
            "found": False,
            "reason": f"数据库不存在: {db_path}"
        }

    candidate = semantic_query_to_label(query_text)

    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    candidate_instance = candidate.replace(" ", "_")

    # chair_2 / bottle_1 这种优先按实例名精确查询
    if "_" in candidate_instance:
        cur.execute("""
            SELECT
                id,
                label,
                label_norm,
                instance_id,
                instance_name,
                observe_count,
                best_confidence,
                first_seen_time,
                last_seen_time,

                best_bbox_area,
                best_bbox_x1,
                best_bbox_y1,
                best_bbox_x2,
                best_bbox_y2,

                rep_camera_x,
                rep_camera_y,
                rep_camera_z,

                rep_map_x,
                rep_map_y,
                rep_map_z,

                camera_frame,
                map_frame,
                rep_image_path
            FROM semantic_objects
            WHERE instance_name = ?
            LIMIT 1
        """, (candidate_instance,))
        rows = cur.fetchall()
    else:
        cur.execute("""
            SELECT
                id,
                label,
                label_norm,
                instance_id,
                instance_name,
                observe_count,
                best_confidence,
                first_seen_time,
                last_seen_time,

                best_bbox_area,
                best_bbox_x1,
                best_bbox_y1,
                best_bbox_x2,
                best_bbox_y2,

                rep_camera_x,
                rep_camera_y,
                rep_camera_z,

                rep_map_x,
                rep_map_y,
                rep_map_z,

                camera_frame,
                map_frame,
                rep_image_path
            FROM semantic_objects
            WHERE label_norm LIKE ?
        """, (f"%{candidate}%",))
        rows = cur.fetchall()

    conn.close()

    if not rows:
        return {
            "found": False,
            "query": query_text,
            "candidate": candidate,
            "reason": "数据库中没有匹配的语义目标实例"
        }

    candidates = []

    for row in rows:
        (
            object_id,
            label,
            label_norm,
            instance_id,
            instance_name,
            observe_count,
            best_confidence,
            first_seen_time,
            last_seen_time,

            best_bbox_area,
            best_bbox_x1,
            best_bbox_y1,
            best_bbox_x2,
            best_bbox_y2,

            rep_camera_x,
            rep_camera_y,
            rep_camera_z,

            rep_map_x,
            rep_map_y,
            rep_map_z,

            camera_frame,
            row_map_frame,
            rep_image_path
        ) = row

        can_navigate = rep_map_x is not None and rep_map_y is not None

        robot_distance = None
        if robot_pose is not None and can_navigate:
            robot_distance = dist2d(robot_pose[0], robot_pose[1], rep_map_x, rep_map_y)

        candidates.append({
            "object_id": object_id,
            "label": label,
            "label_norm": label_norm,
            "instance_id": instance_id,
            "instance_name": instance_name,
            "observe_count": observe_count,
            "best_confidence": best_confidence,
            "first_seen_time": first_seen_time,
            "last_seen_time": last_seen_time,

            "best_bbox": {
                "area": best_bbox_area,
                "x1": best_bbox_x1,
                "y1": best_bbox_y1,
                "x2": best_bbox_x2,
                "y2": best_bbox_y2,
            },

            "representative_camera_position": {
                "x": rep_camera_x,
                "y": rep_camera_y,
                "z": rep_camera_z,
                "frame_id": camera_frame
            },

            "representative_map_position": {
                "x": rep_map_x,
                "y": rep_map_y,
                "z": rep_map_z,
                "frame_id": row_map_frame
            },

            "navigation_position": {
                "x": rep_map_x,
                "y": rep_map_y,
                "z": rep_map_z if rep_map_z is not None else 0.0,
                "frame_id": row_map_frame or map_frame,
                "source": "representative_largest_bbox"
            },

            "can_navigate": can_navigate,
            "robot_distance": robot_distance,
            "representative_image_path": rep_image_path
        })

    navigable = [c for c in candidates if c["can_navigate"]]

    selected = None

    if navigable:
        if select_policy == "nearest" and robot_pose is not None:
            selected = min(
                navigable,
                key=lambda c: float("inf") if c["robot_distance"] is None else c["robot_distance"]
            )
        elif select_policy == "confidence":
            selected = max(navigable, key=lambda c: c["best_confidence"] or 0.0)
        elif select_policy == "bbox":
            selected = max(navigable, key=lambda c: c["best_bbox"]["area"] or 0.0)
        elif select_policy == "latest":
            selected = max(navigable, key=lambda c: c["last_seen_time"] or "")
        else:
            selected = max(navigable, key=lambda c: c["last_seen_time"] or "")
    else:
        selected = candidates[0]

    return {
        "found": True,
        "query": query_text,
        "candidate": candidate,
        "select_policy": select_policy,
        "selected": selected,
        "all_candidates": candidates,
        "count": len(candidates),
        "navigable_count": len(navigable)
    }


def publish_query_target(args):
    robot_pose = None

    if args.select_policy == "nearest":
        try:
            robot_pose = get_robot_pose_in_map_once(
                map_frame=args.map_frame,
                base_frame=args.base_frame
            )
        except Exception:
            robot_pose = None

        if robot_pose is None:
            print("没有获取到机器人 map 位姿，nearest 策略将退化为 latest。")

    result = query_database(
        db_path=args.db_path,
        query_text=args.query,
        select_policy=args.select_policy,
        robot_pose=robot_pose,
        map_frame=args.map_frame
    )

    print(json.dumps(result, ensure_ascii=False, indent=2))

    if not result.get("found", False):
        return

    selected = result.get("selected", None)
    if selected is None:
        print("没有可用目标。")
        return

    if not selected.get("can_navigate", False):
        print("选中的目标没有有效 map 坐标，不能直接导航。")
        print("请确认 SLAM 阶段运行时 TF 链 map -> camera_color_optical_frame 是通的。")
        return

    nav = selected["navigation_position"]

    rclpy.init()
    node = rclpy.create_node("semantic_instance_bbox_query_target_publisher")

    pub = node.create_publisher(PoseStamped, args.target_pose_topic, 10)

    msg = PoseStamped()
    msg.header.frame_id = nav["frame_id"] or args.map_frame
    msg.pose.position.x = float(nav["x"])
    msg.pose.position.y = float(nav["y"])
    msg.pose.position.z = float(nav["z"]) if nav["z"] is not None else 0.0
    msg.pose.orientation.w = 1.0

    node.get_logger().info(
        f"Publishing semantic target to {args.target_pose_topic}: "
        f"instance={selected['instance_name']}, "
        f"x={msg.pose.position.x:.2f}, "
        f"y={msg.pose.position.y:.2f}, "
        f"z={msg.pose.position.z:.2f}, "
        f"frame={msg.header.frame_id}"
    )

    for _ in range(args.publish_count):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(args.publish_interval)

    node.destroy_node()
    rclpy.shutdown()


# ============================================================
# 参数
# ============================================================

def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="YOLOE semantic instance database builder with bbox-area representative rule."
    )

    parser.add_argument(
        "--mode",
        type=str,
        default="build",
        choices=["build", "query", "goto"],
        help="build: 建库；query: 查询数据库；goto: 查询并发布 /yoloe/target_pose"
    )

    parser.add_argument("--query", type=str, default="")
    parser.add_argument("--image-topic", type=str, default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", type=str, default="/camera/depth/image_raw")
    parser.add_argument("--color-info-topic", type=str, default="/camera/color/camera_info")
    parser.add_argument("--depth-info-topic", type=str, default="/camera/depth/camera_info")

    parser.add_argument(
        "--weights-path",
        type=str,
        default="/home/robot2/hzf/2D_robot_ros2_final/src/bringup/weights/yoloe-26s-seg-pf.pt"
    )

    parser.add_argument(
        "--classes",
        type=str,
        default="chair,bottle,cup,person,table,box,door,backpack,keyboard,mouse,phone",
        help="需要保存的类别。设为空字符串 --classes '' 则保存全部类别。"
    )

    parser.add_argument("--min-confidence", type=float, default=0.35)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=5.0)
    parser.add_argument("--roi-size", type=int, default=15)
    parser.add_argument("--sample-period", type=float, default=1.0)
    parser.add_argument("--show-window", action="store_true")

    parser.add_argument(
        "--same-instance-distance",
        type=float,
        default=0.6,
        help="同 label 且 map 距离小于该值，认为是同一个实例。单位 m。"
    )

    parser.add_argument(
        "--db-path",
        type=str,
        default="/home/robot2/semantic_objects_instances_bbox.db"
    )

    parser.add_argument("--image-save-dir", type=str, default="/home/robot2/semantic_images")
    parser.add_argument("--map-frame", type=str, default="map")
    parser.add_argument("--base-frame", type=str, default="base_link")

    parser.add_argument(
        "--target-pose-topic",
        type=str,
        default="/yoloe/target_pose"
    )

    parser.add_argument("--publish-count", type=int, default=10)
    parser.add_argument("--publish-interval", type=float, default=0.1)

    parser.add_argument(
        "--select-policy",
        type=str,
        default="nearest",
        choices=["nearest", "latest", "confidence", "bbox"],
        help="query/goto 多实例选择策略。nearest 需要 map->base_frame TF；bbox 选择代表 bbox 最大的实例。"
    )

    return parser


def main():
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args()

    if args.mode == "query":
        if not args.query:
            print("query 模式需要输入 --query")
            print("例如：python3 semantic_yoloe_db_runtime_instances_bbox.py --mode query --query '椅子'")
            return

        robot_pose = None
        if args.select_policy == "nearest":
            try:
                robot_pose = get_robot_pose_in_map_once(
                    map_frame=args.map_frame,
                    base_frame=args.base_frame
                )
            except Exception:
                robot_pose = None

        result = query_database(
            db_path=args.db_path,
            query_text=args.query,
            select_policy=args.select_policy,
            robot_pose=robot_pose,
            map_frame=args.map_frame
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return

    if args.mode == "goto":
        if not args.query:
            print("goto 模式需要输入 --query")
            print("例如：python3 semantic_yoloe_db_runtime_instances_bbox.py --mode goto --query '椅子'")
            return

        publish_query_target(args)
        return

    rclpy.init(args=ros_args)

    node = SemanticYoloeInstanceDatabaseBuilder(args)

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
