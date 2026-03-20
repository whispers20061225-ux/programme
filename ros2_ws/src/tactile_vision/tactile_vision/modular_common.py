from __future__ import annotations

import base64
import json
from typing import Any, Optional

import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField

try:
    import open3d as o3d
except Exception:  # noqa: BLE001
    o3d = None


def message_stamp_to_seconds(msg: Image) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0.0
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9


def decode_color_image(msg: Image) -> Optional[np.ndarray]:
    height = int(getattr(msg, "height", 0) or 0)
    width = int(getattr(msg, "width", 0) or 0)
    if height <= 0 or width <= 0:
        return None

    encoding = str(getattr(msg, "encoding", "") or "").lower()
    data = np.frombuffer(getattr(msg, "data", b""), dtype=np.uint8)

    if encoding in ("rgb8", "bgr8"):
        expected = height * width * 3
        if data.size < expected:
            return None
        image = data[:expected].reshape((height, width, 3))
        return image[:, :, ::-1] if encoding == "bgr8" else image

    if encoding in ("rgba8", "bgra8"):
        expected = height * width * 4
        if data.size < expected:
            return None
        image = data[:expected].reshape((height, width, 4))
        if encoding == "bgra8":
            image = image[:, :, [2, 1, 0, 3]]
        return image[:, :, :3]

    if encoding in ("mono8", "8uc1"):
        expected = height * width
        if data.size < expected:
            return None
        mono = data[:expected].reshape((height, width))
        return np.repeat(mono[:, :, np.newaxis], 3, axis=2)

    return None


def decode_depth_image(msg: Image) -> Optional[np.ndarray]:
    height = int(getattr(msg, "height", 0) or 0)
    width = int(getattr(msg, "width", 0) or 0)
    if height <= 0 or width <= 0:
        return None

    encoding = str(getattr(msg, "encoding", "") or "").lower()
    if encoding == "32fc1":
        data = np.frombuffer(getattr(msg, "data", b""), dtype=np.float32)
        expected = height * width
        if data.size < expected:
            return None
        return data[:expected].reshape((height, width))

    if encoding == "16uc1":
        data = np.frombuffer(getattr(msg, "data", b""), dtype=np.uint16)
        expected = height * width
        if data.size < expected:
            return None
        return data[:expected].reshape((height, width)).astype(np.float32) / 1000.0

    return None


def decode_mono8_image(msg: Image) -> Optional[np.ndarray]:
    height = int(getattr(msg, "height", 0) or 0)
    width = int(getattr(msg, "width", 0) or 0)
    if height <= 0 or width <= 0:
        return None

    encoding = str(getattr(msg, "encoding", "") or "").lower()
    if encoding not in ("mono8", "8uc1"):
        return None

    data = np.frombuffer(getattr(msg, "data", b""), dtype=np.uint8)
    expected = height * width
    if data.size < expected:
        return None
    return data[:expected].reshape((height, width))


def clamp_int(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, int(value)))


def make_xyz_cloud(points_xyz: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.height = 1

    if points_xyz.size == 0:
        cloud.width = 0
        cloud.row_step = 0
        cloud.is_dense = True
        cloud.data = b""
        return cloud

    points = np.asarray(points_xyz, dtype=np.float32).reshape((-1, 3))
    cloud.width = int(points.shape[0])
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud


def make_mono8_image(mask_u8: np.ndarray, frame_id: str, stamp) -> Image:
    image = Image()
    image.header.frame_id = frame_id
    image.header.stamp = stamp
    mask = np.asarray(mask_u8, dtype=np.uint8)
    if mask.ndim != 2:
        mask = mask.reshape((-1, 1))
    image.height = int(mask.shape[0])
    image.width = int(mask.shape[1])
    image.encoding = "mono8"
    image.is_bigendian = 0
    image.step = int(mask.shape[1])
    image.data = mask.tobytes()
    return image


def make_rgb8_image(image_rgb: np.ndarray, frame_id: str, stamp) -> Image:
    image = Image()
    image.header.frame_id = frame_id
    image.header.stamp = stamp
    rgb = np.asarray(image_rgb, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("rgb image must have shape HxWx3")
    image.height = int(rgb.shape[0])
    image.width = int(rgb.shape[1])
    image.encoding = "rgb8"
    image.is_bigendian = 0
    image.step = int(rgb.shape[1] * 3)
    image.data = rgb.tobytes()
    return image


def quaternion_to_rotation_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_xyzw.tolist()
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def voxel_downsample(points_xyz: np.ndarray, voxel_size_m: float) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float32).reshape((-1, 3))
    if points.size == 0 or voxel_size_m <= 1e-6:
        return points
    if o3d is not None:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        downsampled = point_cloud.voxel_down_sample(float(voxel_size_m))
        return np.asarray(downsampled.points, dtype=np.float32)
    quantized = np.floor(points / float(voxel_size_m)).astype(np.int64)
    _, unique_indices = np.unique(quantized, axis=0, return_index=True)
    unique_indices.sort()
    return points[unique_indices]


def statistical_outlier_filter(
    points_xyz: np.ndarray,
    nb_neighbors: int,
    std_ratio: float,
) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=np.float32).reshape((-1, 3))
    if (
        points.size == 0 or
        o3d is None or
        int(points.shape[0]) < max(8, int(nb_neighbors) + 1)
    ):
        return points
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    filtered_cloud, _ = point_cloud.remove_statistical_outlier(
        nb_neighbors=max(4, int(nb_neighbors)),
        std_ratio=max(0.1, float(std_ratio)),
    )
    filtered_points = np.asarray(filtered_cloud.points, dtype=np.float32)
    if filtered_points.size == 0:
        return points
    return filtered_points


def open3d_available() -> bool:
    return o3d is not None


def encode_image_to_base64_jpeg(image_rgb: np.ndarray, jpeg_quality: int) -> str:
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    success, encoded = cv2.imencode(
        ".jpg",
        image_bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not success:
        raise ValueError("failed to encode frame as JPEG")
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def encode_png_base64(image: np.ndarray) -> str:
    array = np.asarray(image)
    success, encoded = cv2.imencode(".png", array)
    if not success:
        raise ValueError("failed to encode image as PNG")
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def encode_depth_m_to_base64_png(depth_m: np.ndarray) -> str:
    depth = np.asarray(depth_m, dtype=np.float32)
    valid_mask = np.isfinite(depth) & (depth > 0.0)
    depth_mm = np.zeros(depth.shape, dtype=np.uint16)
    if np.any(valid_mask):
        depth_mm[valid_mask] = np.clip(
            np.rint(depth[valid_mask] * 1000.0),
            0,
            np.iinfo(np.uint16).max,
        ).astype(np.uint16)
    return encode_png_base64(depth_mm)


def decode_png_base64_mask(mask_png_b64: str) -> Optional[np.ndarray]:
    try:
        encoded = base64.b64decode(mask_png_b64)
    except Exception:  # noqa: BLE001
        return None
    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None
    return image


def decode_png_base64_image(
    png_b64: str,
    flags: int = cv2.IMREAD_UNCHANGED,
) -> Optional[np.ndarray]:
    try:
        encoded = base64.b64decode(png_b64)
    except Exception:  # noqa: BLE001
        return None
    return cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), flags)


def extract_message_text(payload: dict[str, Any]) -> str:
    choices = payload.get("choices") or []
    if not choices:
        raise ValueError("response missing choices")

    first_choice = choices[0] or {}
    message = first_choice.get("message") or {}
    content = message.get("content")

    if isinstance(content, str):
        return content.strip()

    if isinstance(content, list):
        parts: list[str] = []
        for item in content:
            if not isinstance(item, dict):
                continue
            text = item.get("text")
            if isinstance(text, str) and text.strip():
                parts.append(text.strip())
        if parts:
            return "\n".join(parts)

    text = first_choice.get("text")
    if isinstance(text, str):
        return text.strip()

    raise ValueError("response missing textual content")


def extract_first_json_object(text: str) -> dict[str, Any]:
    candidate = str(text or "").strip()
    if not candidate:
        raise ValueError("empty model response")

    decoder = json.JSONDecoder()
    for index, char in enumerate(candidate):
        if char != "{":
            continue
        try:
            obj, _ = decoder.raw_decode(candidate[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            return obj

    raise ValueError("no JSON object found in model response")


def coerce_point(value: Any, width: int, height: int) -> Optional[list[int]]:
    if value is None or not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    try:
        x = int(round(float(value[0])))
        y = int(round(float(value[1])))
    except (TypeError, ValueError):
        return None
    x = max(0, min(width - 1, x))
    y = max(0, min(height - 1, y))
    return [x, y]


def coerce_bbox(value: Any, width: int, height: int) -> Optional[list[int]]:
    if value is None or not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        x1 = int(round(float(value[0])))
        y1 = int(round(float(value[1])))
        x2 = int(round(float(value[2])))
        y2 = int(round(float(value[3])))
    except (TypeError, ValueError):
        return None

    x1 = max(0, min(width - 1, x1))
    y1 = max(0, min(height - 1, y1))
    x2 = max(0, min(width - 1, x2))
    y2 = max(0, min(height - 1, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def compact_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
