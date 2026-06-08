#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import json
from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple, Optional

import numpy as np


@dataclass
class SemanticObject:
    object_id: int
    class_id: int
    label: str
    num_points: int
    center: List[float]
    min_bound: List[float]
    max_bound: List[float]
    extent: List[float]
    color_mean: List[int]

    def to_dict(self) -> Dict:
        return asdict(self)


def _as_int_list(value) -> List[int]:
    if value is None:
        return []

    if isinstance(value, str):
        if value.strip() == "":
            return []
        return [int(x.strip()) for x in value.split(",") if x.strip()]

    if isinstance(value, (list, tuple, set)):
        return [int(x) for x in value]

    return [int(value)]


def _as_str_list(value) -> List[str]:
    if value is None:
        return []

    if isinstance(value, str):
        if value.strip() == "":
            return []
        return [x.strip().lower() for x in value.split(",") if x.strip()]

    if isinstance(value, (list, tuple, set)):
        return [str(x).strip().lower() for x in value]

    return [str(value).strip().lower()]


def _safe_label(value, default: str = "unknown") -> str:
    try:
        if value is None:
            return default
        # numpy string scalar
        return str(value)
    except Exception:
        return default


def _get_class_id_to_label_from_npz(data) -> Dict[int, str]:
    """
    Read class_id -> label mapping from new npz format.

    Expected:
      unique_class_ids
      unique_labels
    """
    mapping: Dict[int, str] = {}

    if "unique_class_ids" in data.files and "unique_labels" in data.files:
        for cid, label in zip(data["unique_class_ids"], data["unique_labels"]):
            mapping[int(cid)] = _safe_label(label, f"unknown_{int(cid)}")

    return mapping


def _normalize_colors(colors, n_points: int) -> np.ndarray:
    if colors is None:
        return np.full((n_points, 3), 200, dtype=np.uint8)

    colors = np.asarray(colors)

    if colors.ndim != 2 or colors.shape[0] != n_points or colors.shape[1] < 3:
        return np.full((n_points, 3), 200, dtype=np.uint8)

    colors = colors[:, :3]

    if colors.size > 0 and colors.max() <= 1.0:
        return np.clip(colors * 255.0, 0, 255).astype(np.uint8)

    return np.clip(colors, 0, 255).astype(np.uint8)


def _make_object_from_points(
    object_id: int,
    class_id: int,
    label: str,
    pts_obj: np.ndarray,
    colors_obj: Optional[np.ndarray],
    fallback_center: Optional[np.ndarray] = None,
    fallback_num_points: Optional[int] = None,
) -> SemanticObject:
    """
    Build SemanticObject. Prefer real points for bbox/color.
    If points are unavailable, use fallback center.
    """

    if pts_obj is not None and pts_obj.shape[0] > 0:
        center = pts_obj.mean(axis=0)
        min_bound = pts_obj.min(axis=0)
        max_bound = pts_obj.max(axis=0)
        extent = max_bound - min_bound
        num_points = int(pts_obj.shape[0])

        if colors_obj is not None and colors_obj.shape[0] == pts_obj.shape[0]:
            color_mean = colors_obj[:, :3].mean(axis=0)
        else:
            color_mean = np.asarray([200, 200, 200], dtype=np.float32)

    else:
        if fallback_center is None:
            fallback_center = np.asarray([0.0, 0.0, 0.0], dtype=np.float32)

        center = np.asarray(fallback_center, dtype=np.float32)
        min_bound = center - np.asarray([0.05, 0.05, 0.05], dtype=np.float32)
        max_bound = center + np.asarray([0.05, 0.05, 0.05], dtype=np.float32)
        extent = max_bound - min_bound
        num_points = int(fallback_num_points) if fallback_num_points is not None else 0
        color_mean = np.asarray([200, 200, 200], dtype=np.float32)

    return SemanticObject(
        object_id=int(object_id),
        class_id=int(class_id),
        label=str(label),
        num_points=int(num_points),
        center=[float(x) for x in center.tolist()],
        min_bound=[float(x) for x in min_bound.tolist()],
        max_bound=[float(x) for x in max_bound.tolist()],
        extent=[float(x) for x in extent.tolist()],
        color_mean=[
            int(x)
            for x in np.clip(color_mean, 0, 255).astype(np.uint8).tolist()
        ],
    )


def load_semantic_objects_from_npz(
    npz_path: str,
    min_points: int = 20,
    ignore_class_ids=None,
    ignore_labels=None,
    ignore_object_ids=None,
) -> List[SemanticObject]:
    """
    Read pyslam-exported semantic_dense_map_latest.npz.

    New recommended npz fields:
      points:             Nx3 float32
      colors:             Nx3 uint8
      class_ids:          N int32
      object_ids:         N int32
      labels:             N string

      unique_class_ids:   M int32
      unique_labels:      M string

      object_ids_unique:  K int32
      object_class_ids:   K int32
      object_labels:      K string
      object_centers:     Kx3 float32
      object_num_points:  K int32

    Returns:
      List[SemanticObject] with object_id, class_id, label, center, bbox, point count.
    """

    ignore_class_ids = set(_as_int_list(ignore_class_ids))
    ignore_labels = set(_as_str_list(ignore_labels))
    ignore_object_ids = set(_as_int_list(ignore_object_ids))

    data = np.load(npz_path)

    class_id_to_label = _get_class_id_to_label_from_npz(data)

    points = data["points"].astype(np.float32) if "points" in data.files else None
    colors = data["colors"] if "colors" in data.files else None
    class_ids = data["class_ids"].astype(np.int32) if "class_ids" in data.files else None
    object_ids = data["object_ids"].astype(np.int32) if "object_ids" in data.files else None

    if points is not None:
        colors = _normalize_colors(colors, points.shape[0])

    objects: List[SemanticObject] = []

    # ============================================================
    # New format: object-level arrays exist.
    # This is the preferred path for your current npz.
    # ============================================================
    has_object_level = all(
        k in data.files
        for k in [
            "object_ids_unique",
            "object_class_ids",
            "object_labels",
            "object_centers",
            "object_num_points",
        ]
    )

    if has_object_level:
        object_ids_unique = data["object_ids_unique"].astype(np.int32)
        object_class_ids = data["object_class_ids"].astype(np.int32)
        object_labels = data["object_labels"]
        object_centers = data["object_centers"].astype(np.float32)
        object_num_points = data["object_num_points"].astype(np.int32)

        for oid, cid, label, center, npts in zip(
            object_ids_unique,
            object_class_ids,
            object_labels,
            object_centers,
            object_num_points,
        ):
            oid = int(oid)
            cid = int(cid)
            label = _safe_label(label, class_id_to_label.get(cid, f"unknown_{cid}"))

            if oid in ignore_object_ids:
                continue

            if cid in ignore_class_ids:
                continue

            if label.lower() in ignore_labels:
                continue

            if int(npts) < int(min_points):
                continue

            pts_obj = None
            colors_obj = None

            # If point-level object_ids exist, compute bbox/color from real points.
            if points is not None and object_ids is not None:
                mask_obj = object_ids == oid
                if mask_obj.any():
                    pts_obj = points[mask_obj]
                    colors_obj = colors[mask_obj] if colors is not None else None

            obj = _make_object_from_points(
                object_id=oid,
                class_id=cid,
                label=label,
                pts_obj=pts_obj,
                colors_obj=colors_obj,
                fallback_center=center,
                fallback_num_points=int(npts),
            )

            objects.append(obj)

        objects.sort(key=lambda o: (o.label, -o.num_points, o.object_id))
        return objects

    # ============================================================
    # Old format fallback:
    # Only points/colors/class_ids/object_ids exist.
    # Group by object_id and infer majority class/label.
    # ============================================================
    required = ["points", "class_ids", "object_ids"]
    missing = [k for k in required if k not in data.files]
    if missing:
        raise ValueError(f"missing required npz keys: {missing}")

    if points is None or class_ids is None or object_ids is None:
        raise ValueError("invalid npz: points/class_ids/object_ids cannot be None")

    if not (points.shape[0] == colors.shape[0] == class_ids.shape[0] == object_ids.shape[0]):
        raise ValueError(
            "points/colors/class_ids/object_ids length mismatch: "
            f"{points.shape[0]}, {colors.shape[0]}, {class_ids.shape[0]}, {object_ids.shape[0]}"
        )

    point_labels = data["labels"] if "labels" in data.files else None

    for oid in np.unique(object_ids):
        oid = int(oid)

        if oid in ignore_object_ids:
            continue

        mask_obj = object_ids == oid
        pts_obj = points[mask_obj]
        cls_obj = class_ids[mask_obj]
        colors_obj = colors[mask_obj]

        if pts_obj.shape[0] == 0:
            continue

        unique_cls, counts = np.unique(cls_obj, return_counts=True)
        majority_class = int(unique_cls[np.argmax(counts)])

        if majority_class in ignore_class_ids:
            continue

        if pts_obj.shape[0] < int(min_points):
            continue

        if point_labels is not None:
            labels_obj = point_labels[mask_obj]
            unique_labels, label_counts = np.unique(labels_obj, return_counts=True)
            label = _safe_label(unique_labels[np.argmax(label_counts)], f"unknown_{majority_class}")
        else:
            label = class_id_to_label.get(majority_class, f"unknown_{majority_class}")

        if label.lower() in ignore_labels:
            continue

        obj = _make_object_from_points(
            object_id=oid,
            class_id=majority_class,
            label=label,
            pts_obj=pts_obj,
            colors_obj=colors_obj,
        )

        objects.append(obj)

    objects.sort(key=lambda o: (o.label, -o.num_points, o.object_id))
    return objects


def compute_approach_goal(
    center_xyz: List[float],
    approach_distance: float = 0.8,
    map_origin_xy: Tuple[float, float] = (0.0, 0.0),
) -> Tuple[float, float, float]:
    """
    Compute a simple 2D navigation goal near an object.

    Rule:
      object_center = (cx, cy)
      direction = object_center - map_origin
      goal = object_center - approach_distance * unit(direction)
      yaw faces the object center

    Later you can replace map_origin_xy with current robot pose from TF.
    """

    cx = float(center_xyz[0])
    cy = float(center_xyz[1])

    ox, oy = float(map_origin_xy[0]), float(map_origin_xy[1])
    vx = cx - ox
    vy = cy - oy
    norm = math.hypot(vx, vy)

    if norm < 1e-6:
        gx, gy = cx, cy
        yaw = 0.0
    else:
        ux = vx / norm
        uy = vy / norm
        gx = cx - float(approach_distance) * ux
        gy = cy - float(approach_distance) * uy
        yaw = math.atan2(cy - gy, cx - gx)

    return float(gx), float(gy), float(yaw)


def objects_to_json(objects: List[SemanticObject]) -> str:
    return json.dumps([o.to_dict() for o in objects], ensure_ascii=False, indent=2)