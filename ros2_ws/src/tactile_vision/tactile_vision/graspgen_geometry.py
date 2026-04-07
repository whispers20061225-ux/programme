from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float32).reshape((3,))
    length = float(np.linalg.norm(values))
    if length <= 1e-6:
        return np.zeros((3,), dtype=np.float32)
    return values / length


def build_grasp_axes(
    rotation: np.ndarray,
    *,
    fallback_approach_direction: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    matrix = np.asarray(rotation, dtype=np.float32).reshape((3, 3))
    closing_direction = normalize_vector(matrix[:, 0])
    approach_direction = normalize_vector(matrix[:, 2])

    if not np.any(closing_direction):
        closing_direction = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    if not np.any(approach_direction):
        if fallback_approach_direction is not None:
            approach_direction = normalize_vector(fallback_approach_direction)
        if not np.any(approach_direction):
            approach_direction = np.array([0.0, 0.0, 1.0], dtype=np.float32)

    lateral_direction = normalize_vector(np.cross(approach_direction, closing_direction))
    if not np.any(lateral_direction):
        fallback_axis = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        if abs(float(np.dot(fallback_axis, approach_direction))) > 0.95:
            fallback_axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        lateral_direction = normalize_vector(np.cross(approach_direction, fallback_axis))

    closing_direction = normalize_vector(np.cross(lateral_direction, approach_direction))
    if not np.any(closing_direction):
        closing_direction = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    return closing_direction.astype(np.float32), lateral_direction.astype(np.float32), approach_direction.astype(np.float32)


def _load_yaml_dict(path: Path) -> dict[str, Any]:
    from omegaconf import OmegaConf

    raw_cfg = OmegaConf.to_container(OmegaConf.load(str(path)), resolve=True)
    if not isinstance(raw_cfg, dict):
        raise ValueError(f"invalid GraspGen config: {path}")
    return raw_cfg


def _extract_gripper_name(raw_cfg: dict[str, Any]) -> str:
    gripper_name = str(raw_cfg.get("gripper_name") or "").strip()
    if gripper_name:
        return gripper_name
    data_cfg = raw_cfg.get("data")
    if isinstance(data_cfg, dict):
        return str(data_cfg.get("gripper_name") or "").strip()
    return ""


def _looks_like_geometry_cfg(raw_cfg: dict[str, Any]) -> bool:
    if isinstance(raw_cfg.get("contact_points"), list) and raw_cfg.get("contact_points"):
        return True
    if raw_cfg.get("approach_direction") is not None:
        return True
    if raw_cfg.get("width") is not None:
        return True
    if raw_cfg.get("depth") is not None:
        return True
    return False


def resolve_gripper_geometry_config(
    *,
    repo_root: Path,
    gripper_config_path: Optional[Path],
    gripper_name: str = "",
) -> tuple[Path, dict[str, Any]]:
    loaded: list[tuple[Path, dict[str, Any]]] = []
    candidate_paths: list[Path] = []

    metadata_path = (
        Path(gripper_config_path).expanduser().resolve()
        if gripper_config_path is not None
        else None
    )
    if metadata_path is not None:
        candidate_paths.append(metadata_path)
    if gripper_name:
        candidate_paths.append((repo_root / "config" / "grippers" / f"{gripper_name}.yaml").resolve())

    for path in candidate_paths:
        if not path.is_file():
            continue
        raw_cfg = _load_yaml_dict(path)
        loaded.append((path, raw_cfg))
        if _looks_like_geometry_cfg(raw_cfg):
            return path, raw_cfg

        nested_gripper_name = _extract_gripper_name(raw_cfg)
        if nested_gripper_name:
            nested_path = (repo_root / "config" / "grippers" / f"{nested_gripper_name}.yaml").resolve()
            if nested_path.is_file() and nested_path not in candidate_paths:
                candidate_paths.append(nested_path)

    if loaded:
        checked = ", ".join(str(path) for path, _ in loaded)
        raise ValueError(
            "unable to resolve gripper geometry config with contact_points/depth; "
            f"checked: {checked}"
        )
    if metadata_path is not None:
        raise FileNotFoundError(f"missing GraspGen config: {metadata_path}")
    raise ValueError("missing GraspGen gripper configuration metadata")


def load_gripper_geometry(
    *,
    repo_root: Path,
    gripper_config_path: Optional[Path],
    gripper_name: str = "",
) -> dict[str, Any]:
    resolved_path, raw_cfg = resolve_gripper_geometry_config(
        repo_root=repo_root,
        gripper_config_path=gripper_config_path,
        gripper_name=gripper_name,
    )

    approach_direction = np.asarray(
        raw_cfg.get("approach_direction") or [0.0, 0.0, 1.0],
        dtype=np.float32,
    ).reshape((3,))
    if not np.any(approach_direction):
        approach_direction = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    approach_direction = normalize_vector(approach_direction)

    contact_locations: list[np.ndarray] = []
    for entry in list(raw_cfg.get("contact_points") or []):
        if not isinstance(entry, dict):
            continue
        values = entry.get("location")
        if isinstance(values, (list, tuple)) and len(values) >= 3:
            contact_locations.append(np.asarray(values[:3], dtype=np.float32).reshape((3,)))

    width = max(0.0, float(raw_cfg.get("width") or 0.0))
    depth = float(raw_cfg.get("depth") or 0.0)
    if len(contact_locations) >= 2:
        contact_point_1_local = contact_locations[0]
        contact_point_2_local = contact_locations[1]
    else:
        half_width = 0.5 * width if width > 1e-6 else 0.04
        contact_depth = depth if abs(depth) > 1e-6 else 0.0
        contact_point_1_local = np.array([-half_width, 0.0, contact_depth], dtype=np.float32)
        contact_point_2_local = np.array([half_width, 0.0, contact_depth], dtype=np.float32)

    return {
        "path": resolved_path,
        "gripper_name": _extract_gripper_name(raw_cfg) or gripper_name,
        "approach_direction_local": approach_direction,
        "contact_point_1_local": contact_point_1_local.astype(np.float32),
        "contact_point_2_local": contact_point_2_local.astype(np.float32),
        "width": width,
        "depth": depth,
        "raw": raw_cfg,
    }


def grasp_pose_to_contacts(
    grasp_pose: np.ndarray,
    gripper_geometry: dict[str, Any],
) -> dict[str, np.ndarray]:
    matrix = np.asarray(grasp_pose, dtype=np.float32).reshape((4, 4))
    rotation = np.asarray(matrix[:3, :3], dtype=np.float32).reshape((3, 3))
    translation = np.asarray(matrix[:3, 3], dtype=np.float32).reshape((3,))

    contact_point_1 = rotation @ gripper_geometry["contact_point_1_local"] + translation
    contact_point_2 = rotation @ gripper_geometry["contact_point_2_local"] + translation
    grasp_center = 0.5 * (contact_point_1 + contact_point_2)
    closing_direction, lateral_direction, approach_direction = build_grasp_axes(
        rotation,
        fallback_approach_direction=rotation @ gripper_geometry["approach_direction_local"],
    )

    return {
        "rotation": rotation,
        "translation": translation,
        "contact_point_1": contact_point_1.astype(np.float32),
        "contact_point_2": contact_point_2.astype(np.float32),
        "grasp_center": grasp_center.astype(np.float32),
        "closing_direction": closing_direction.astype(np.float32),
        "lateral_direction": lateral_direction.astype(np.float32),
        "approach_direction": approach_direction.astype(np.float32),
    }


def _select_surface_contact(
    object_points: np.ndarray,
    local_points: np.ndarray,
    *,
    expected_x: float,
    side_sign: float,
    x_sigma: float,
    radial_sigma: float,
) -> np.ndarray:
    side_mask = local_points[:, 0] * float(side_sign) >= 0.0
    if not np.any(side_mask):
        return np.empty((0, 3), dtype=np.float32)

    local_side = local_points[side_mask]
    world_side = object_points[side_mask]
    radial_sq = np.square(local_side[:, 1]) + np.square(local_side[:, 2])
    score = (
        np.square((local_side[:, 0] - float(expected_x)) / float(x_sigma))
        + radial_sq / float(radial_sigma * radial_sigma)
    )
    best_index = int(np.argmin(score))
    return np.asarray(world_side[best_index], dtype=np.float32).reshape((3,))


def reconstruct_surface_contacts(
    grasp_pose: np.ndarray,
    gripper_geometry: dict[str, Any],
    object_points: np.ndarray,
    *,
    max_gripper_width_m: float,
) -> dict[str, np.ndarray | float]:
    template = grasp_pose_to_contacts(grasp_pose, gripper_geometry)
    points = np.asarray(object_points, dtype=np.float32).reshape((-1, 3))
    max_width = max(0.0, float(max_gripper_width_m))
    template_width = float(np.linalg.norm(template["contact_point_2"] - template["contact_point_1"]))
    half_width = 0.5 * (max_width if max_width > 1e-6 else template_width)

    if points.size == 0:
        return {
            **template,
            "surface_contact_point_1": template["contact_point_1"].copy(),
            "surface_contact_point_2": template["contact_point_2"].copy(),
            "surface_grasp_center": template["grasp_center"].copy(),
            "surface_width_m": template_width,
            "effective_grasp_width_m": min(template_width, max_width) if max_width > 1e-6 else template_width,
        }

    center = np.asarray(template["grasp_center"], dtype=np.float32).reshape((3,))
    closing_direction = np.asarray(template["closing_direction"], dtype=np.float32).reshape((3,))
    lateral_direction = np.asarray(template["lateral_direction"], dtype=np.float32).reshape((3,))
    approach_direction = np.asarray(template["approach_direction"], dtype=np.float32).reshape((3,))

    expected_contact_point_1 = center - closing_direction * float(half_width)
    expected_contact_point_2 = center + closing_direction * float(half_width)

    offsets = points - center.reshape((1, 3))
    local_points = np.column_stack(
        [
            offsets @ closing_direction,
            offsets @ lateral_direction,
            offsets @ approach_direction,
        ]
    ).astype(np.float32)

    x_sigma = max(0.006, 0.5 * float(half_width), 0.25 * template_width)
    radial_sigma = max(0.008, 0.5 * float(half_width))
    contact_point_1 = _select_surface_contact(
        points,
        local_points,
        expected_x=-float(half_width),
        side_sign=-1.0,
        x_sigma=x_sigma,
        radial_sigma=radial_sigma,
    )
    contact_point_2 = _select_surface_contact(
        points,
        local_points,
        expected_x=float(half_width),
        side_sign=1.0,
        x_sigma=x_sigma,
        radial_sigma=radial_sigma,
    )

    if contact_point_1.size == 0:
        contact_point_1 = expected_contact_point_1.astype(np.float32)
    if contact_point_2.size == 0:
        contact_point_2 = expected_contact_point_2.astype(np.float32)

    surface_grasp_center = 0.5 * (contact_point_1 + contact_point_2)
    surface_width_m = float(np.linalg.norm(contact_point_2 - contact_point_1))
    effective_grasp_width_m = (
        min(surface_width_m, max_width) if max_width > 1e-6 else surface_width_m
    )

    return {
        **template,
        "surface_contact_point_1": contact_point_1.astype(np.float32),
        "surface_contact_point_2": contact_point_2.astype(np.float32),
        "surface_grasp_center": surface_grasp_center.astype(np.float32),
        "surface_width_m": surface_width_m,
        "effective_grasp_width_m": float(effective_grasp_width_m),
    }
