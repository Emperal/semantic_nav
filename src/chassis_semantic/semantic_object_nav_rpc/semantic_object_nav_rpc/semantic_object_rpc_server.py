#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import json
import os
import socket
import struct
import threading
import time
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from bsonrpc import JSONRpc, request, service_class

from .semantic_map_utils import (
    SemanticObject,
    load_semantic_objects_from_npz,
)

try:
    import open3d as o3d
except Exception as e:
    o3d = None
    OPEN3D_IMPORT_ERROR = str(e)
else:
    OPEN3D_IMPORT_ERROR = None


def normalize_label(label: str) -> str:
    return str(label).strip().lower()


@service_class
class SemanticObjectRPCServices:
    def __init__(self, node):
        self.node = node

    @request
    def reload_map(self):
        return self.node.reload_map()

    @request
    def recompute_icp(self):
        return self.node.recompute_icp()

    @request
    def list_objects(self):
        return self.node.list_objects()

    @request
    def list_labels(self):
        return self.node.list_labels()

    @request
    def find_by_class(self, class_id: int):
        return self.node.find_by_class(int(class_id))

    @request
    def find_by_label(self, label: str):
        return self.node.find_by_label(str(label))

    @request
    def get_object(self, object_id: int):
        return self.node.get_object(int(object_id))

    @request
    def publish_object_cloud(self, object_id: int):
        return self.node.publish_object_cloud_rpc(int(object_id))

    @request
    def goto_object(self, object_id: int, approach_distance: float = -1.0):
        return self.node.goto_object(int(object_id), float(approach_distance))

    @request
    def goto_class(self, class_id: int, approach_distance: float = -1.0):
        return self.node.goto_class(int(class_id), float(approach_distance))

    @request
    def goto_label(self, label: str, approach_distance: float = -1.0):
        return self.node.goto_label(str(label), float(approach_distance))

    @request
    def candidate_goals_for_object(self, object_id: int, approach_distance: float = -1.0):
        return self.node.candidate_goals_for_object(int(object_id), float(approach_distance))

    @request
    def get_state(self):
        return self.node.get_state()

    @request
    def get_nav_state(self):
        return self.node.query_nav_state()


class SemanticObjectRPCServer(Node):
    def __init__(self):
        super().__init__("semantic_object_rpc_server")

        # ============================================================
        # Semantic map input
        # ============================================================
        self.declare_parameter(
            "npz_path",
            "/home/robot2/hzf/1asemantic_nav/lib/pyslam/results/metrics_20260528_220648/semantic_dense_map_latest.npz",
        )
        self.declare_parameter(
            "semantic_ply_path",
            "/home/robot2/hzf/1asemantic_nav/lib/pyslam/results/metrics_20260528_220648/semantic_dense_map_latest.ply",
        )

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("min_points", 20)

        self.declare_parameter("ignore_class_ids", "0")
        self.declare_parameter("ignore_labels", "")
        self.declare_parameter("ignore_object_ids", "")
        self.declare_parameter("reload_period", 1.0)

        # ============================================================
        # LIO-SAM / aligned map cloud topic
        # ============================================================
        self.declare_parameter("map_cloud_topic", "/aligned_cloud")
        self.declare_parameter("map_cloud_max_points", 250000)
        self.declare_parameter("map_cloud_min_points", 1000)

        # ============================================================
        # Navigation goal generation
        # ============================================================
        self.declare_parameter("approach_distance", 0.8)
        self.declare_parameter("map_origin_x", 0.0)
        self.declare_parameter("map_origin_y", 0.0)

        # ============================================================
        # Candidate goal generation and filtering
        # ============================================================
        self.declare_parameter("candidate_num_samples", 36)
        self.declare_parameter("candidate_robot_radius", 0.35)
        self.declare_parameter("candidate_safety_margin", 0.25)
        self.declare_parameter("candidate_extra_distance", 0.0)
        self.declare_parameter("use_nav_goal_check", True)
        self.declare_parameter("allow_unchecked_goal_fallback", False)
        self.declare_parameter("max_return_candidates", 12)

        # ============================================================
        # Target object pointcloud visualization
        # ============================================================
        self.declare_parameter("publish_target_cloud_on_goto", True)
        self.declare_parameter("target_object_cloud_topic", "/semantic_target_object_cloud")
        self.declare_parameter("target_object_info_topic", "/semantic_target_object_info")
        self.declare_parameter("target_object_cloud_max_points", 100000)

        # ============================================================
        # Semantic RPC server
        # ============================================================
        self.declare_parameter("semantic_rpc_host", "127.0.0.1")
        self.declare_parameter("semantic_rpc_port", 6001)

        # ============================================================
        # Navigation RPC target
        # ============================================================
        self.declare_parameter("nav_rpc_host", "127.0.0.1")
        self.declare_parameter("nav_rpc_port", 6000)
        self.declare_parameter("nav_rpc_timeout_sec", 3.0)

        # ============================================================
        # ICP alignment
        # ============================================================
        self.declare_parameter("use_icp_alignment", True)

        # 推荐 true：ICP source 直接用 NPZ 里的 points，避免 Open3D 读到带 string label 的 PLY 失败。
        self.declare_parameter("icp_source_use_npz", True)

        self.declare_parameter(
            "icp_transform_path",
            "/home/robot2/hzf/1asemantic_nav/src/chassis_semantic/semantic_to_map_icp.npy",
        )

        self.declare_parameter("icp_recompute", True)
        self.declare_parameter("icp_voxel_size", 0.10)
        self.declare_parameter("icp_max_correspondence_distance", 0.50)
        self.declare_parameter("icp_min_fitness", 0.05)

        self.declare_parameter("icp_initial_x", 0.0)
        self.declare_parameter("icp_initial_y", 0.0)
        self.declare_parameter("icp_initial_z", 0.0)
        self.declare_parameter("icp_initial_roll", 0.0)
        self.declare_parameter("icp_initial_pitch", 0.0)
        self.declare_parameter("icp_initial_yaw", 0.0)

        # ============================================================
        # Read params
        # ============================================================
        self.npz_path = self.get_parameter("npz_path").value
        self.semantic_ply_path = self.get_parameter("semantic_ply_path").value
        self.frame_id = self.get_parameter("frame_id").value
        self.min_points = int(self.get_parameter("min_points").value)

        self.ignore_class_ids = self.get_parameter("ignore_class_ids").value
        self.ignore_labels = self.get_parameter("ignore_labels").value
        self.ignore_object_ids = self.get_parameter("ignore_object_ids").value
        self.reload_period = float(self.get_parameter("reload_period").value)

        self.map_cloud_topic = self.get_parameter("map_cloud_topic").value
        self.map_cloud_max_points = int(self.get_parameter("map_cloud_max_points").value)
        self.map_cloud_min_points = int(self.get_parameter("map_cloud_min_points").value)

        self.approach_distance = float(self.get_parameter("approach_distance").value)
        self.map_origin_x = float(self.get_parameter("map_origin_x").value)
        self.map_origin_y = float(self.get_parameter("map_origin_y").value)

        self.candidate_num_samples = int(self.get_parameter("candidate_num_samples").value)
        self.candidate_robot_radius = float(self.get_parameter("candidate_robot_radius").value)
        self.candidate_safety_margin = float(self.get_parameter("candidate_safety_margin").value)
        self.candidate_extra_distance = float(self.get_parameter("candidate_extra_distance").value)
        self.use_nav_goal_check = bool(self.get_parameter("use_nav_goal_check").value)
        self.allow_unchecked_goal_fallback = bool(
            self.get_parameter("allow_unchecked_goal_fallback").value
        )
        self.max_return_candidates = int(self.get_parameter("max_return_candidates").value)

        self.publish_target_cloud_on_goto = bool(
            self.get_parameter("publish_target_cloud_on_goto").value
        )
        self.target_object_cloud_topic = self.get_parameter("target_object_cloud_topic").value
        self.target_object_info_topic = self.get_parameter("target_object_info_topic").value
        self.target_object_cloud_max_points = int(
            self.get_parameter("target_object_cloud_max_points").value
        )

        self.semantic_rpc_host = self.get_parameter("semantic_rpc_host").value
        self.semantic_rpc_port = int(self.get_parameter("semantic_rpc_port").value)

        self.nav_rpc_host = self.get_parameter("nav_rpc_host").value
        self.nav_rpc_port = int(self.get_parameter("nav_rpc_port").value)
        self.nav_rpc_timeout_sec = float(self.get_parameter("nav_rpc_timeout_sec").value)

        self.use_icp_alignment = bool(self.get_parameter("use_icp_alignment").value)
        self.icp_source_use_npz = bool(self.get_parameter("icp_source_use_npz").value)
        self.icp_transform_path = self.get_parameter("icp_transform_path").value
        self.icp_recompute = bool(self.get_parameter("icp_recompute").value)
        self.icp_voxel_size = float(self.get_parameter("icp_voxel_size").value)
        self.icp_max_correspondence_distance = float(
            self.get_parameter("icp_max_correspondence_distance").value
        )
        self.icp_min_fitness = float(self.get_parameter("icp_min_fitness").value)

        self.icp_initial_x = float(self.get_parameter("icp_initial_x").value)
        self.icp_initial_y = float(self.get_parameter("icp_initial_y").value)
        self.icp_initial_z = float(self.get_parameter("icp_initial_z").value)
        self.icp_initial_roll = float(self.get_parameter("icp_initial_roll").value)
        self.icp_initial_pitch = float(self.get_parameter("icp_initial_pitch").value)
        self.icp_initial_yaw = float(self.get_parameter("icp_initial_yaw").value)

        # ============================================================
        # Runtime state
        # ============================================================
        self.T_map_pyslam = np.eye(4, dtype=np.float64)
        self.icp_fitness = -1.0
        self.icp_inlier_rmse = -1.0
        self.icp_status = "NOT_RUN"

        self.latest_map_cloud_msg: Optional[PointCloud2] = None
        self.latest_map_cloud_frame_id = ""
        self.latest_map_cloud_stamp_sec = 0.0
        self.latest_map_cloud_points_count = 0
        self.map_cloud_ready = False

        self.objects: List[SemanticObject] = []
        self.last_mtime = 0.0
        self.lock = threading.RLock()
        self.state = "INIT"

        self.objects_pub = self.create_publisher(String, "/semantic_objects", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/semantic_object_markers", 10)

        target_cloud_qos = QoSProfile(depth=1)
        target_cloud_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        target_cloud_qos.reliability = ReliabilityPolicy.RELIABLE

        self.target_object_cloud_pub = self.create_publisher(
            PointCloud2,
            self.target_object_cloud_topic,
            target_cloud_qos,
        )

        self.target_object_info_pub = self.create_publisher(
            String,
            self.target_object_info_topic,
            10,
        )

        self.map_cloud_sub = self.create_subscription(
            PointCloud2,
            self.map_cloud_topic,
            self.map_cloud_callback,
            10,
        )

        self.init_icp_alignment_from_cache_if_possible()
        self.reload_map()

        self.timer = self.create_timer(max(0.1, self.reload_period), self.timer_callback)

        self.rpc_thread = threading.Thread(target=self.start_rpc_server, daemon=True)
        self.rpc_thread.start()

        self.get_logger().info("Semantic object RPC server started")
        self.get_logger().info(f"NPZ path: {self.npz_path}")
        self.get_logger().info(f"Semantic PLY path: {self.semantic_ply_path}")
        self.get_logger().info(f"Map cloud topic: {self.map_cloud_topic}")
        self.get_logger().info(f"Target object cloud topic: {self.target_object_cloud_topic}")
        self.get_logger().info(f"Semantic RPC: {self.semantic_rpc_host}:{self.semantic_rpc_port}")
        self.get_logger().info(f"Nav RPC target: {self.nav_rpc_host}:{self.nav_rpc_port}")

    # ============================================================
    # LIO-SAM map cloud topic callback
    # ============================================================

    def map_cloud_callback(self, msg: PointCloud2):
        with self.lock:
            self.latest_map_cloud_msg = msg
            self.latest_map_cloud_frame_id = msg.header.frame_id
            self.latest_map_cloud_stamp_sec = (
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            )
            self.map_cloud_ready = True

        # 只在第一次收到点云时自动尝试 ICP。
        # 不要把 FAILED 放在这里，否则失败后每帧点云都会疯狂重算。
        if self.use_icp_alignment and self.icp_recompute and self.icp_status in [
            "NOT_RUN",
            "WAITING_FOR_MAP_CLOUD",
        ]:
            self.get_logger().info(
                f"Received map cloud from topic {self.map_cloud_topic}, start ICP..."
            )
            self.recompute_icp()

    # ============================================================
    # ICP alignment
    # ============================================================

    def init_icp_alignment_from_cache_if_possible(self):
        self.T_map_pyslam = np.eye(4, dtype=np.float64)
        self.icp_fitness = -1.0
        self.icp_inlier_rmse = -1.0

        if not self.use_icp_alignment:
            self.icp_status = "DISABLED"
            self.get_logger().warn(
                "ICP alignment disabled. Semantic object centers remain in pyslam coordinates."
            )
            return

        # 优先读取已有矩阵。读取 .npy 不需要 Open3D。
        if os.path.exists(self.icp_transform_path) and not self.icp_recompute:
            try:
                self.T_map_pyslam = np.load(self.icp_transform_path).astype(np.float64)
                self.icp_status = "LOADED"
                self.get_logger().info(
                    f"Loaded ICP transform from {self.icp_transform_path}:\n"
                    f"{self.T_map_pyslam}"
                )
                return
            except Exception as e:
                self.icp_status = "LOAD_FAILED"
                self.get_logger().warn(f"Failed to load ICP transform: {e}")

        if o3d is None:
            self.icp_status = "OPEN3D_NOT_INSTALLED"
            self.get_logger().error(
                f"open3d is not installed or failed to import. Error: {OPEN3D_IMPORT_ERROR}"
            )
            return

        self.icp_status = "WAITING_FOR_MAP_CLOUD"
        self.get_logger().info(
            "ICP will be computed after receiving map cloud topic."
        )

    def make_transform_from_xyz_rpy(self, x, y, z, roll, pitch, yaw) -> np.ndarray:
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
            dtype=np.float64,
        )
        ry = np.array(
            [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
            dtype=np.float64,
        )
        rz = np.array(
            [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        r = rz @ ry @ rx

        t = np.eye(4, dtype=np.float64)
        t[:3, :3] = r
        t[0, 3] = x
        t[1, 3] = y
        t[2, 3] = z
        return t

    def pointcloud2_to_open3d(self, msg: PointCloud2):
        pts = []

        for p in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                pts.append([x, y, z])

        if len(pts) == 0:
            raise RuntimeError("PointCloud2 has no valid XYZ points")

        pts = np.asarray(pts, dtype=np.float64)

        if pts.shape[0] > self.map_cloud_max_points:
            idx = np.random.choice(
                pts.shape[0],
                size=self.map_cloud_max_points,
                replace=False,
            )
            pts = pts[idx]

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts)

        return cloud

    def load_source_cloud_from_npz(self):
        """
        Load pyslam dense semantic points from NPZ and convert to Open3D PointCloud.

        This avoids Open3D PLY parsing failure when PLY has custom string fields.
        """
        if not os.path.exists(self.npz_path):
            raise RuntimeError(f"npz_path does not exist: {self.npz_path}")

        data = np.load(self.npz_path)

        if "points" not in data.files:
            raise RuntimeError(f"npz has no 'points' array: {self.npz_path}")

        points = data["points"].astype(np.float64)

        if points.ndim != 2 or points.shape[1] < 3:
            raise RuntimeError(f"invalid npz points shape: {points.shape}")

        points = points[:, :3]
        points = points[np.isfinite(points).all(axis=1)]

        if points.shape[0] == 0:
            raise RuntimeError("npz points are empty after filtering invalid points")

        if points.shape[0] > self.map_cloud_max_points:
            idx = np.random.choice(
                points.shape[0],
                size=self.map_cloud_max_points,
                replace=False,
            )
            points = points[idx]

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        return cloud

    def recompute_icp(self):
        self.T_map_pyslam = np.eye(4, dtype=np.float64)
        self.icp_fitness = -1.0
        self.icp_inlier_rmse = -1.0
        self.icp_status = "FAILED"

        if not self.use_icp_alignment:
            self.icp_status = "DISABLED"
            return {
                "ok": False,
                "message": "ICP alignment disabled",
                "T_map_pyslam": self.T_map_pyslam.tolist(),
            }

        if o3d is None:
            self.icp_status = "OPEN3D_NOT_INSTALLED"
            return {
                "ok": False,
                "message": f"open3d import failed: {OPEN3D_IMPORT_ERROR}",
                "T_map_pyslam": self.T_map_pyslam.tolist(),
            }

        with self.lock:
            msg = self.latest_map_cloud_msg

        if msg is None:
            self.icp_status = "WAITING_FOR_MAP_CLOUD"
            return {
                "ok": False,
                "message": f"waiting for PointCloud2 topic: {self.map_cloud_topic}",
                "T_map_pyslam": self.T_map_pyslam.tolist(),
            }

        try:
            if self.icp_source_use_npz:
                source = self.load_source_cloud_from_npz()
            else:
                if not os.path.exists(self.semantic_ply_path):
                    raise RuntimeError(f"semantic_ply_path does not exist: {self.semantic_ply_path}")

                source = o3d.io.read_point_cloud(self.semantic_ply_path)
                if source.is_empty():
                    raise RuntimeError(f"source semantic PLY is empty: {self.semantic_ply_path}")

            target = self.pointcloud2_to_open3d(msg)
            if target.is_empty():
                raise RuntimeError("target map cloud from topic is empty")

            self.latest_map_cloud_points_count = len(target.points)

            if len(target.points) < self.map_cloud_min_points:
                raise RuntimeError(
                    f"target map cloud has too few points: "
                    f"{len(target.points)} < {self.map_cloud_min_points}"
                )

            self.T_map_pyslam = self.compute_icp_transform_from_clouds(
                source=source,
                target=target,
                voxel_size=self.icp_voxel_size,
                max_corr_dist=self.icp_max_correspondence_distance,
            )

            transform_dir = os.path.dirname(self.icp_transform_path)
            if transform_dir:
                os.makedirs(transform_dir, exist_ok=True)

            np.save(self.icp_transform_path, self.T_map_pyslam)
            self.icp_status = "OK"

            self.get_logger().info(
                f"Saved ICP transform to {self.icp_transform_path}:\n"
                f"{self.T_map_pyslam}"
            )

            self.reload_map()

            return {
                "ok": True,
                "message": "ICP recomputed from map cloud topic",
                "icp_source": "npz" if self.icp_source_use_npz else "ply",
                "map_cloud_topic": self.map_cloud_topic,
                "map_cloud_frame_id": self.latest_map_cloud_frame_id,
                "map_cloud_points": self.latest_map_cloud_points_count,
                "fitness": self.icp_fitness,
                "inlier_rmse": self.icp_inlier_rmse,
                "T_map_pyslam": self.T_map_pyslam.tolist(),
            }

        except Exception as e:
            self.icp_status = "FAILED"
            self.T_map_pyslam = np.eye(4, dtype=np.float64)
            self.get_logger().error(f"ICP alignment failed: {e}")
            return {
                "ok": False,
                "message": str(e),
                "T_map_pyslam": self.T_map_pyslam.tolist(),
            }

    def compute_icp_transform_from_clouds(
        self,
        source,
        target,
        voxel_size: float = 0.10,
        max_corr_dist: float = 0.50,
    ) -> np.ndarray:
        source.remove_non_finite_points()
        target.remove_non_finite_points()

        self.get_logger().info(
            f"ICP source semantic points={len(source.points)}"
        )
        self.get_logger().info(
            f"ICP target map cloud points={len(target.points)}"
        )

        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)

        if source_down.is_empty():
            raise RuntimeError("source_down is empty after voxel_down_sample")

        if target_down.is_empty():
            raise RuntimeError("target_down is empty after voxel_down_sample")

        self.get_logger().info(
            f"ICP downsampled source={len(source_down.points)}, "
            f"target={len(target_down.points)}, voxel_size={voxel_size}"
        )

        normal_radius = voxel_size * 3.0

        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_radius,
                max_nn=30,
            )
        )
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_radius,
                max_nn=30,
            )
        )

        init = self.make_transform_from_xyz_rpy(
            self.icp_initial_x,
            self.icp_initial_y,
            self.icp_initial_z,
            self.icp_initial_roll,
            self.icp_initial_pitch,
            self.icp_initial_yaw,
        )

        self.get_logger().info(f"ICP initial transform:\n{init}")

        reg = o3d.pipelines.registration.registration_icp(
            source_down,
            target_down,
            max_corr_dist,
            init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100,
            ),
        )

        self.icp_fitness = float(reg.fitness)
        self.icp_inlier_rmse = float(reg.inlier_rmse)

        self.get_logger().info(
            f"ICP result: fitness={reg.fitness:.6f}, "
            f"inlier_rmse={reg.inlier_rmse:.6f}"
        )
        self.get_logger().info(f"ICP transform T_map_pyslam:\n{reg.transformation}")

        if reg.fitness < self.icp_min_fitness:
            self.get_logger().warn(
                f"ICP fitness too low: {reg.fitness:.6f} < {self.icp_min_fitness}. "
                f"Transform may be unreliable."
            )

        return np.asarray(reg.transformation, dtype=np.float64)

    def transform_point_pyslam_to_map(self, point_xyz):
        p = np.asarray(
            [float(point_xyz[0]), float(point_xyz[1]), float(point_xyz[2]), 1.0],
            dtype=np.float64,
        )
        q = self.T_map_pyslam @ p
        return [float(q[0]), float(q[1]), float(q[2])]

    def transform_points_pyslam_to_map(self, points_xyz: np.ndarray) -> np.ndarray:
        points_xyz = np.asarray(points_xyz, dtype=np.float64)

        if points_xyz.size == 0:
            return points_xyz.reshape(0, 3).astype(np.float32)

        if not (self.use_icp_alignment and self.icp_status in ["OK", "LOADED"]):
            return points_xyz[:, :3].astype(np.float32)

        ones = np.ones((points_xyz.shape[0], 1), dtype=np.float64)
        points_h = np.hstack([points_xyz[:, :3], ones])
        points_map = (self.T_map_pyslam @ points_h.T).T[:, :3]

        return points_map.astype(np.float32)

    def transform_bbox_pyslam_to_map(self, min_bound, max_bound):
        mn = np.asarray(min_bound, dtype=np.float64)
        mx = np.asarray(max_bound, dtype=np.float64)

        corners = []
        for x in [mn[0], mx[0]]:
            for y in [mn[1], mx[1]]:
                for z in [mn[2], mx[2]]:
                    corners.append([x, y, z, 1.0])

        corners = np.asarray(corners, dtype=np.float64)
        transformed = (self.T_map_pyslam @ corners.T).T[:, :3]

        new_min = transformed.min(axis=0)
        new_max = transformed.max(axis=0)
        new_extent = new_max - new_min

        return (
            [float(x) for x in new_min.tolist()],
            [float(x) for x in new_max.tolist()],
            [float(x) for x in new_extent.tolist()],
        )

    def transform_object_pyslam_to_map(self, obj: SemanticObject) -> SemanticObject:
        out = copy.deepcopy(obj)

        out.center = self.transform_point_pyslam_to_map(obj.center)

        new_min, new_max, new_extent = self.transform_bbox_pyslam_to_map(
            obj.min_bound,
            obj.max_bound,
        )

        out.min_bound = new_min
        out.max_bound = new_max
        out.extent = new_extent

        return out

    # ============================================================
    # Timers and loading
    # ============================================================

    def timer_callback(self):
        try:
            if os.path.exists(self.npz_path):
                mtime = os.path.getmtime(self.npz_path)
                if mtime != self.last_mtime:
                    self.reload_map()

            self.publish_objects()

        except Exception as e:
            self.get_logger().error(f"timer_callback error: {e}")

    def reload_map(self):
        if not os.path.exists(self.npz_path):
            self.state = "NO_MAP"
            msg = f"npz file not found: {self.npz_path}"
            self.get_logger().warn(msg)
            return {"ok": False, "state": self.state, "message": msg}

        try:
            objects_pyslam = load_semantic_objects_from_npz(
                self.npz_path,
                min_points=self.min_points,
                ignore_class_ids=self.ignore_class_ids,
                ignore_labels=self.ignore_labels,
                ignore_object_ids=self.ignore_object_ids,
            )

            if self.use_icp_alignment and self.icp_status in ["OK", "LOADED"]:
                objects = [
                    self.transform_object_pyslam_to_map(obj)
                    for obj in objects_pyslam
                ]
            else:
                objects = objects_pyslam

            with self.lock:
                self.objects = objects
                self.last_mtime = os.path.getmtime(self.npz_path)
                self.state = "READY"

            msg = f"loaded {len(objects)} semantic objects"
            self.get_logger().info(msg)

            return {
                "ok": True,
                "state": self.state,
                "num_objects": len(objects),
                "use_icp_alignment": self.use_icp_alignment,
                "icp_status": self.icp_status,
                "icp_fitness": self.icp_fitness,
                "icp_inlier_rmse": self.icp_inlier_rmse,
                "map_cloud_topic": self.map_cloud_topic,
                "map_cloud_frame_id": self.latest_map_cloud_frame_id,
                "T_map_pyslam": self.T_map_pyslam.tolist(),
                "objects": [o.to_dict() for o in objects],
            }

        except Exception as e:
            self.state = "ERROR"
            self.get_logger().error(f"failed to reload map: {e}")
            return {"ok": False, "state": self.state, "message": str(e)}

    # ============================================================
    # Query APIs
    # ============================================================

    def list_objects(self):
        with self.lock:
            return [o.to_dict() for o in self.objects]

    def list_labels(self):
        with self.lock:
            table = {}

            for obj in self.objects:
                key = normalize_label(obj.label)

                if key not in table:
                    table[key] = {
                        "label": obj.label,
                        "class_id": int(obj.class_id),
                        "num_objects": 0,
                        "object_ids": [],
                        "total_points": 0,
                    }

                table[key]["num_objects"] += 1
                table[key]["object_ids"].append(int(obj.object_id))
                table[key]["total_points"] += int(obj.num_points)

        return list(table.values())

    def find_by_class(self, class_id: int):
        with self.lock:
            return [o.to_dict() for o in self.objects if o.class_id == int(class_id)]

    def find_by_label(self, label: str):
        target = normalize_label(label)

        with self.lock:
            candidates = [
                o.to_dict()
                for o in self.objects
                if normalize_label(o.label) == target
            ]

        return candidates

    def get_object(self, object_id: int):
        with self.lock:
            for obj in self.objects:
                if obj.object_id == int(object_id):
                    return obj.to_dict()

        return None

    def _choose_best_object_by_class(self, class_id: int) -> Optional[SemanticObject]:
        with self.lock:
            candidates = [o for o in self.objects if o.class_id == int(class_id)]

        if not candidates:
            return None

        candidates.sort(key=lambda o: o.num_points, reverse=True)
        return candidates[0]

    def _choose_best_object_by_label(self, label: str) -> Optional[SemanticObject]:
        target = normalize_label(label)

        with self.lock:
            candidates = [
                o for o in self.objects
                if normalize_label(o.label) == target
            ]

        if not candidates:
            return None

        candidates.sort(key=lambda o: o.num_points, reverse=True)
        return candidates[0]

    # ============================================================
    # Target object pointcloud visualization
    # ============================================================

    def pack_rgb_float(self, r: int, g: int, b: int) -> float:
        r = int(np.clip(r, 0, 255))
        g = int(np.clip(g, 0, 255))
        b = int(np.clip(b, 0, 255))
        rgb_uint32 = (r << 16) | (g << 8) | b
        return struct.unpack("f", struct.pack("I", rgb_uint32))[0]

    def load_object_points_from_npz(self, object_id: int):
        if not os.path.exists(self.npz_path):
            raise RuntimeError(f"npz_path does not exist: {self.npz_path}")

        data = np.load(self.npz_path)

        for key in ["points", "object_ids"]:
            if key not in data.files:
                raise RuntimeError(f"npz file missing key '{key}': {self.npz_path}")

        points = data["points"].astype(np.float32)
        object_ids = data["object_ids"].astype(np.int32)

        if points.ndim != 2 or points.shape[1] < 3:
            raise RuntimeError(f"invalid points shape: {points.shape}")

        if object_ids.shape[0] != points.shape[0]:
            raise RuntimeError(
                f"points/object_ids length mismatch: "
                f"{points.shape[0]} vs {object_ids.shape[0]}"
            )

        mask = object_ids == int(object_id)

        if not np.any(mask):
            raise RuntimeError(f"object_id={object_id} has no points in npz")

        pts = points[mask, :3]

        if "colors" in data.files:
            colors_all = np.asarray(data["colors"])

            if colors_all.shape[0] == points.shape[0]:
                colors = colors_all[mask, :3]

                if colors.size > 0 and colors.max() <= 1.0:
                    colors = np.clip(colors * 255.0, 0, 255).astype(np.uint8)
                else:
                    colors = np.clip(colors, 0, 255).astype(np.uint8)
            else:
                colors = np.full((pts.shape[0], 3), [255, 80, 80], dtype=np.uint8)
        else:
            colors = np.full((pts.shape[0], 3), [255, 80, 80], dtype=np.uint8)

        valid = np.isfinite(pts).all(axis=1)

        pts = pts[valid]
        colors = colors[valid]

        if pts.shape[0] == 0:
            raise RuntimeError(f"object_id={object_id} points are empty after filtering")

        if pts.shape[0] > self.target_object_cloud_max_points:
            idx = np.random.choice(
                pts.shape[0],
                size=self.target_object_cloud_max_points,
                replace=False,
            )
            pts = pts[idx]
            colors = colors[idx]

        return pts.astype(np.float32), colors.astype(np.uint8)

    def make_xyzrgb_pointcloud2(
        self,
        points_xyz: np.ndarray,
        colors_rgb: np.ndarray,
        frame_id: str,
    ) -> PointCloud2:
        points_xyz = np.asarray(points_xyz, dtype=np.float32)
        colors_rgb = np.asarray(colors_rgb, dtype=np.uint8)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []

        for p, c in zip(points_xyz, colors_rgb):
            rgb = self.pack_rgb_float(int(c[0]), int(c[1]), int(c[2]))
            cloud_data.append([float(p[0]), float(p[1]), float(p[2]), rgb])

        return point_cloud2.create_cloud(header, fields, cloud_data)

    def publish_target_object_cloud(self, obj: SemanticObject):
        try:
            raw_points, colors = self.load_object_points_from_npz(obj.object_id)
            points_map = self.transform_points_pyslam_to_map(raw_points)

            msg = self.make_xyzrgb_pointcloud2(
                points_xyz=points_map,
                colors_rgb=colors,
                frame_id=self.frame_id,
            )

            self.target_object_cloud_pub.publish(msg)

            info = {
                "ok": True,
                "topic": self.target_object_cloud_topic,
                "frame_id": self.frame_id,
                "object_id": int(obj.object_id),
                "class_id": int(obj.class_id),
                "label": str(obj.label),
                "num_points": int(points_map.shape[0]),
                "center": [float(x) for x in obj.center],
                "icp_status": self.icp_status,
            }

            info_msg = String()
            info_msg.data = json.dumps(info, ensure_ascii=False)
            self.target_object_info_pub.publish(info_msg)

            self.get_logger().info(
                f"Published target object cloud: "
                f"object_id={obj.object_id}, "
                f"label={obj.label}, "
                f"points={points_map.shape[0]}, "
                f"topic={self.target_object_cloud_topic}"
            )

            return info

        except Exception as e:
            self.get_logger().error(f"failed to publish target object cloud: {e}")

            return {
                "ok": False,
                "message": str(e),
                "topic": self.target_object_cloud_topic,
                "object_id": int(obj.object_id),
                "label": str(obj.label),
            }

    def publish_object_cloud_rpc(self, object_id: int):
        obj = None

        with self.lock:
            for item in self.objects:
                if int(item.object_id) == int(object_id):
                    obj = item
                    break

        if obj is None:
            return {
                "ok": False,
                "message": f"no object found for object_id={object_id}",
            }

        return self.publish_target_object_cloud(obj)

    # ============================================================
    # Candidate goal generation and filtering
    # ============================================================

    def get_robot_xy_fallback(self):
        return float(self.map_origin_x), float(self.map_origin_y)

    def point_in_expanded_bbox_2d(self, x, y, obj: SemanticObject, margin: float):
        min_x = float(obj.min_bound[0]) - float(margin)
        min_y = float(obj.min_bound[1]) - float(margin)
        max_x = float(obj.max_bound[0]) + float(margin)
        max_y = float(obj.max_bound[1]) + float(margin)

        return min_x <= float(x) <= max_x and min_y <= float(y) <= max_y

    def compute_candidate_goals_around_object(
        self,
        obj: SemanticObject,
        approach_distance: float,
    ):
        cx = float(obj.center[0])
        cy = float(obj.center[1])

        extent_x = max(float(obj.extent[0]), 0.05)
        extent_y = max(float(obj.extent[1]), 0.05)

        object_radius = 0.5 * float(np.hypot(extent_x, extent_y))

        stand_radius = (
            object_radius
            + float(approach_distance)
            + float(self.candidate_robot_radius)
            + float(self.candidate_safety_margin)
            + float(self.candidate_extra_distance)
        )

        rx, ry = self.get_robot_xy_fallback()

        candidates = []
        n = max(int(self.candidate_num_samples), 8)

        for i in range(n):
            theta = 2.0 * np.pi * float(i) / float(n)

            gx = cx + stand_radius * np.cos(theta)
            gy = cy + stand_radius * np.sin(theta)

            if self.point_in_expanded_bbox_2d(
                gx,
                gy,
                obj,
                margin=self.candidate_robot_radius + self.candidate_safety_margin,
            ):
                continue

            yaw = float(np.arctan2(cy - gy, cx - gx))
            dist_robot = float(np.hypot(gx - rx, gy - ry))

            candidates.append(
                {
                    "x": float(gx),
                    "y": float(gy),
                    "yaw": float(yaw),
                    "score": float(dist_robot),
                    "theta": float(theta),
                    "stand_radius": float(stand_radius),
                    "object_radius": float(object_radius),
                    "mode": "bbox_candidate",
                }
            )

        candidates.sort(key=lambda c: c["score"])
        return candidates

    def call_nav_check_goal(self, x: float, y: float, yaw: float):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.nav_rpc_timeout_sec)
            s.connect((self.nav_rpc_host, self.nav_rpc_port))

            rpc = JSONRpc(s)
            server = rpc.get_peer_proxy()

            result = server.check_goal(float(x), float(y), float(yaw))
            rpc.close()

            return result

        except Exception as e:
            return {
                "ok": False,
                "reachable": False,
                "reason": str(e),
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
            }

    def choose_best_goal_near_object(
        self,
        obj: SemanticObject,
        approach_distance: float,
    ):
        candidates = self.compute_candidate_goals_around_object(
            obj=obj,
            approach_distance=approach_distance,
        )

        if len(candidates) == 0:
            return {
                "ok": False,
                "message": "no candidate goals generated around object",
                "candidates": [],
            }

        checked_candidates = []

        if self.use_nav_goal_check:
            for cand in candidates:
                check = self.call_nav_check_goal(
                    cand["x"],
                    cand["y"],
                    cand["yaw"],
                )

                cand["nav_check"] = check
                checked_candidates.append(cand)

                if bool(check.get("ok", False)) and bool(check.get("reachable", False)):
                    return {
                        "ok": True,
                        "goal": cand,
                        "all_candidates": checked_candidates[: self.max_return_candidates],
                        "selected_by": "first_reachable_nav2_checked_candidate",
                    }

            if self.allow_unchecked_goal_fallback:
                best = candidates[0]
                best["nav_check"] = {
                    "ok": False,
                    "reachable": None,
                    "reason": "all nav checks failed, fallback to nearest candidate",
                }
                return {
                    "ok": True,
                    "goal": best,
                    "all_candidates": checked_candidates[: self.max_return_candidates],
                    "selected_by": "fallback_nearest_candidate_after_nav_check_failed",
                }

            return {
                "ok": False,
                "message": "no reachable candidate found by Nav2 check_goal",
                "all_candidates": checked_candidates[: self.max_return_candidates],
            }

        if self.allow_unchecked_goal_fallback:
            best = candidates[0]
            best["nav_check"] = {
                "ok": False,
                "reachable": None,
                "reason": "nav goal check disabled, using nearest candidate",
            }
            return {
                "ok": True,
                "goal": best,
                "all_candidates": [best],
                "selected_by": "unchecked_nearest_candidate",
            }

        return {
            "ok": False,
            "message": "nav goal check disabled and unchecked fallback is not allowed",
            "all_candidates": candidates[: self.max_return_candidates],
        }

    def candidate_goals_for_object(self, object_id: int, approach_distance: float = -1.0):
        if approach_distance is None or approach_distance < 0:
            approach_distance = self.approach_distance

        obj = None

        with self.lock:
            for item in self.objects:
                if item.object_id == int(object_id):
                    obj = item
                    break

        if obj is None:
            return {
                "ok": False,
                "message": f"no object found for object_id={object_id}",
            }

        candidates = self.compute_candidate_goals_around_object(
            obj=obj,
            approach_distance=approach_distance,
        )

        return {
            "ok": True,
            "object": obj.to_dict(),
            "candidate_count": len(candidates),
            "candidates": candidates[: self.max_return_candidates],
        }

    # ============================================================
    # Navigation APIs
    # ============================================================

    def goto_class(self, class_id: int, approach_distance: float = -1.0):
        obj = self._choose_best_object_by_class(class_id)

        if obj is None:
            return {
                "ok": False,
                "message": f"no object found for class_id={class_id}",
                "state": self.state,
                "available_labels": self.list_labels(),
            }

        return self._goto_object_impl(obj, approach_distance)

    def goto_label(self, label: str, approach_distance: float = -1.0):
        obj = self._choose_best_object_by_label(label)

        if obj is None:
            return {
                "ok": False,
                "message": f"no object found for label={label}",
                "state": self.state,
                "available_labels": self.list_labels(),
            }

        return self._goto_object_impl(obj, approach_distance)

    def goto_object(self, object_id: int, approach_distance: float = -1.0):
        obj = None

        with self.lock:
            for item in self.objects:
                if item.object_id == int(object_id):
                    obj = item
                    break

        if obj is None:
            return {
                "ok": False,
                "message": f"no object found for object_id={object_id}",
                "state": self.state,
                "available_labels": self.list_labels(),
            }

        return self._goto_object_impl(obj, approach_distance)

    def _goto_object_impl(self, obj: SemanticObject, approach_distance: float = -1.0):
        if approach_distance is None or approach_distance < 0:
            approach_distance = self.approach_distance

        if self.use_icp_alignment and self.icp_status not in ["OK", "LOADED"]:
            return {
                "ok": False,
                "message": "ICP alignment is not ready, object center is not in ROS map frame yet",
                "icp_status": self.icp_status,
                "object": obj.to_dict(),
            }

        selection = self.choose_best_goal_near_object(
            obj=obj,
            approach_distance=approach_distance,
        )

        if not bool(selection.get("ok", False)):
            return {
                "ok": False,
                "message": selection.get(
                    "message",
                    "failed to select reachable candidate goal",
                ),
                "object": obj.to_dict(),
                "icp_status": self.icp_status,
                "icp_fitness": self.icp_fitness,
                "candidate_result": selection,
            }

        goal = selection["goal"]

        gx = float(goal["x"])
        gy = float(goal["y"])
        yaw = float(goal["yaw"])

        target_cloud_result = None
        if self.publish_target_cloud_on_goto:
            target_cloud_result = self.publish_target_object_cloud(obj)

        nav_result = self.call_nav_add_goal(gx, gy, yaw)

        return {
            "ok": True,
            "object": obj.to_dict(),
            "target_object_cloud": target_cloud_result,
            "goal": {
                "x": gx,
                "y": gy,
                "yaw": yaw,
                "approach_distance": approach_distance,
                "frame_id": self.frame_id,
                "selected_by": selection.get("selected_by", ""),
                "candidate_score": goal.get("score", 0.0),
                "stand_radius": goal.get("stand_radius", 0.0),
                "object_radius": goal.get("object_radius", 0.0),
                "nav_check": goal.get("nav_check", {}),
            },
            "candidate_count_checked": len(selection.get("all_candidates", [])),
            "checked_candidates": selection.get("all_candidates", []),
            "icp_status": self.icp_status,
            "icp_fitness": self.icp_fitness,
            "nav_rpc_result": nav_result,
        }

    def call_nav_add_goal(self, x: float, y: float, yaw: float):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.nav_rpc_timeout_sec)
            s.connect((self.nav_rpc_host, self.nav_rpc_port))

            rpc = JSONRpc(s)
            server = rpc.get_peer_proxy()

            result = server.add_goal(float(x), float(y), float(yaw))
            rpc.close()

            return result

        except Exception as e:
            self.get_logger().error(f"failed to call nav add_goal: {e}")
            return {"error": str(e)}

    def query_nav_state(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.nav_rpc_timeout_sec)
            s.connect((self.nav_rpc_host, self.nav_rpc_port))

            rpc = JSONRpc(s)
            server = rpc.get_peer_proxy()

            state = server.get_state()
            rpc.close()

            return state

        except Exception as e:
            return {"error": str(e)}

    def get_state(self):
        return {
            "semantic_state": self.state,
            "npz_path": self.npz_path,
            "semantic_ply_path": self.semantic_ply_path,
            "frame_id": self.frame_id,
            "map_cloud_topic": self.map_cloud_topic,
            "map_cloud_ready": self.map_cloud_ready,
            "map_cloud_frame_id": self.latest_map_cloud_frame_id,
            "map_cloud_stamp_sec": self.latest_map_cloud_stamp_sec,
            "map_cloud_points_count": self.latest_map_cloud_points_count,
            "use_icp_alignment": self.use_icp_alignment,
            "icp_source_use_npz": self.icp_source_use_npz,
            "icp_status": self.icp_status,
            "icp_fitness": self.icp_fitness,
            "icp_inlier_rmse": self.icp_inlier_rmse,
            "icp_transform_path": self.icp_transform_path,
            "T_map_pyslam": self.T_map_pyslam.tolist(),
            "target_cloud": {
                "publish_target_cloud_on_goto": self.publish_target_cloud_on_goto,
                "target_object_cloud_topic": self.target_object_cloud_topic,
                "target_object_info_topic": self.target_object_info_topic,
                "target_object_cloud_max_points": self.target_object_cloud_max_points,
            },
            "candidate_config": {
                "candidate_num_samples": self.candidate_num_samples,
                "candidate_robot_radius": self.candidate_robot_radius,
                "candidate_safety_margin": self.candidate_safety_margin,
                "candidate_extra_distance": self.candidate_extra_distance,
                "use_nav_goal_check": self.use_nav_goal_check,
                "allow_unchecked_goal_fallback": self.allow_unchecked_goal_fallback,
            },
            "num_objects": len(self.list_objects()),
            "labels": self.list_labels(),
            "nav_state": self.query_nav_state(),
        }

    # ============================================================
    # Visualization
    # ============================================================

    def publish_objects(self):
        objects = self.list_objects()

        msg = String()
        msg.data = json.dumps(objects, ensure_ascii=False)
        self.objects_pub.publish(msg)

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.header.stamp = now
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        for obj in objects:
            center = obj["center"]
            extent = obj["extent"]
            color = obj.get("color_mean", [0, 255, 0])
            label = obj.get("label", f"unknown_{obj['class_id']}")

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "semantic_objects"
            marker.id = int(obj["object_id"])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = max(float(extent[0]), 0.05)
            marker.scale.y = max(float(extent[1]), 0.05)
            marker.scale.z = max(float(extent[2]), 0.05)

            marker.color.r = float(color[0]) / 255.0
            marker.color.g = float(color[1]) / 255.0
            marker.color.b = float(color[2]) / 255.0
            marker.color.a = 0.45

            markers.markers.append(marker)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = "semantic_object_labels"
            text.id = int(obj["object_id"]) + 100000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = float(center[0])
            text.pose.position.y = float(center[1])
            text.pose.position.z = float(center[2]) + max(float(extent[2]), 0.2) + 0.1
            text.pose.orientation.w = 1.0

            text.scale.z = 0.25
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0

            text.text = (
                f"{label} "
                f"id={obj['object_id']} "
                f"cls={obj['class_id']} "
                f"n={obj['num_points']}"
            )

            markers.markers.append(text)

        self.marker_pub.publish(markers)

    # ============================================================
    # RPC server
    # ============================================================

    def start_rpc_server(self):
        ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ss.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ss.bind((self.semantic_rpc_host, self.semantic_rpc_port))
        ss.listen(10)

        self.get_logger().info(
            f"semantic object RPC server listening on "
            f"{self.semantic_rpc_host}:{self.semantic_rpc_port}"
        )

        while rclpy.ok():
            try:
                client_sock, addr = ss.accept()
                self.get_logger().info(f"semantic RPC client connected from {addr}")
                JSONRpc(client_sock, SemanticObjectRPCServices(self))

            except Exception as e:
                self.get_logger().error(f"semantic RPC server error: {e}")
                time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticObjectRPCServer()

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