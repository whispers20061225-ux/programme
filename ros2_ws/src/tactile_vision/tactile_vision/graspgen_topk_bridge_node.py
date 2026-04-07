from __future__ import annotations

import os
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool

from tactile_vision.modular_common import make_xyz_cloud


def _stamp_to_signature(msg: PointCloud2) -> str:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0) or 0)
    nanosec = int(getattr(stamp, "nanosec", 0) or 0)
    frame_id = str(getattr(header, "frame_id", "") or "")
    width = int(getattr(msg, "width", 0) or 0)
    data_len = len(getattr(msg, "data", b""))
    return f"{sec}:{nanosec}:{frame_id}:{width}:{data_len}"


def _cloud_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    points = np.array(
        [
            [float(point[0]), float(point[1]), float(point[2])]
            for point in point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ],
        dtype=np.float32,
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return points.reshape((-1, 3))


def _build_pair_line_cloud(
    contact_point_1: np.ndarray,
    contact_point_2: np.ndarray,
    *,
    samples_per_pair: int,
) -> np.ndarray:
    points_1 = np.asarray(contact_point_1, dtype=np.float32).reshape((-1, 3))
    points_2 = np.asarray(contact_point_2, dtype=np.float32).reshape((-1, 3))
    pair_count = min(points_1.shape[0], points_2.shape[0])
    if pair_count <= 0:
        return np.empty((0, 3), dtype=np.float32)

    sample_count = max(2, int(samples_per_pair))
    interpolation = np.linspace(0.0, 1.0, sample_count, dtype=np.float32).reshape((1, -1, 1))
    start = points_1[:pair_count].reshape((pair_count, 1, 3))
    end = points_2[:pair_count].reshape((pair_count, 1, 3))
    line_points = start + (end - start) * interpolation
    return line_points.reshape((-1, 3)).astype(np.float32)


class GraspGenTopKBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("graspgen_topk_bridge_node")

        self.declare_parameter("target_cloud_topic", "/perception/target_cloud")
        self.declare_parameter("target_locked_topic", "/sim/perception/target_locked")
        self.declare_parameter("topk_cloud_topic", "/grasp/graspgen_shadow/topk_cloud")
        self.declare_parameter("topk_pair_markers_topic", "/grasp/graspgen_shadow/topk_pair_markers")
        self.declare_parameter(
            "topk_contact_point_1_topic",
            "/grasp/graspgen_shadow/topk_contact_point_1_cloud",
        )
        self.declare_parameter(
            "topk_contact_point_2_topic",
            "/grasp/graspgen_shadow/topk_contact_point_2_cloud",
        )
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("require_target_locked", False)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("sensor_stale_sec", 6.0)
        self.declare_parameter("retry_same_cloud_sec", 4.0)
        self.declare_parameter("min_points_required", 32)
        self.declare_parameter("max_points", 2048)
        self.declare_parameter("publish_empty_on_failure", False)
        self.declare_parameter("graspgen_python", "/home/whispers/GraspGen/.venv/bin/python")
        self.declare_parameter("graspgen_repo_root", "/home/whispers/GraspGen")
        self.declare_parameter("graspgen_host", "127.0.0.1")
        self.declare_parameter("graspgen_port", 5556)
        self.declare_parameter("request_timeout_sec", 12.0)
        self.declare_parameter("subprocess_timeout_sec", 20.0)
        self.declare_parameter("grasp_threshold", -1.0)
        self.declare_parameter("num_grasps", 200)
        self.declare_parameter("topk_num_grasps", 50)
        self.declare_parameter("min_grasps", 40)
        self.declare_parameter("max_tries", 6)
        self.declare_parameter("remove_outliers", False)
        self.declare_parameter("max_gripper_width_m", 0.06)
        self.declare_parameter("pair_line_width_m", 0.0018)
        self.declare_parameter("pair_point_scale_m", 0.0032)
        self.declare_parameter("line_samples_per_pair", 20)

        self.target_cloud_topic = str(self.get_parameter("target_cloud_topic").value)
        self.target_locked_topic = str(self.get_parameter("target_locked_topic").value)
        self.topk_cloud_topic = str(self.get_parameter("topk_cloud_topic").value)
        self.topk_pair_markers_topic = str(
            self.get_parameter("topk_pair_markers_topic").value
        )
        self.topk_contact_point_1_topic = str(
            self.get_parameter("topk_contact_point_1_topic").value
        )
        self.topk_contact_point_2_topic = str(
            self.get_parameter("topk_contact_point_2_topic").value
        )
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.require_target_locked = bool(self.get_parameter("require_target_locked").value)
        self.publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.sensor_stale_sec = max(0.0, float(self.get_parameter("sensor_stale_sec").value))
        self.retry_same_cloud_sec = max(
            0.0, float(self.get_parameter("retry_same_cloud_sec").value)
        )
        self.min_points_required = max(1, int(self.get_parameter("min_points_required").value))
        self.max_points = max(0, int(self.get_parameter("max_points").value))
        self.publish_empty_on_failure = bool(
            self.get_parameter("publish_empty_on_failure").value
        )
        self.graspgen_python = str(self.get_parameter("graspgen_python").value)
        self.graspgen_repo_root = str(self.get_parameter("graspgen_repo_root").value)
        self.graspgen_host = str(self.get_parameter("graspgen_host").value)
        self.graspgen_port = int(self.get_parameter("graspgen_port").value)
        self.request_timeout_sec = max(
            1.0, float(self.get_parameter("request_timeout_sec").value)
        )
        self.subprocess_timeout_sec = max(
            self.request_timeout_sec + 1.0,
            float(self.get_parameter("subprocess_timeout_sec").value),
        )
        self.grasp_threshold = float(self.get_parameter("grasp_threshold").value)
        self.num_grasps = max(1, int(self.get_parameter("num_grasps").value))
        self.topk_num_grasps = max(1, int(self.get_parameter("topk_num_grasps").value))
        self.min_grasps = max(1, int(self.get_parameter("min_grasps").value))
        self.max_tries = max(1, int(self.get_parameter("max_tries").value))
        self.remove_outliers = bool(self.get_parameter("remove_outliers").value)
        self.max_gripper_width_m = max(
            0.01, float(self.get_parameter("max_gripper_width_m").value)
        )
        self.pair_line_width_m = max(
            0.0005, float(self.get_parameter("pair_line_width_m").value)
        )
        self.pair_point_scale_m = max(
            0.0005, float(self.get_parameter("pair_point_scale_m").value)
        )
        self.line_samples_per_pair = max(
            2, int(self.get_parameter("line_samples_per_pair").value)
        )
        self.helper_script = Path(__file__).with_name("graspgen_topk_helper.py")

        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE
        qos_latched = QoSProfile(depth=1)
        qos_latched.reliability = ReliabilityPolicy.RELIABLE
        qos_latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._latest_target_cloud: Optional[PointCloud2] = None
        self._latest_target_cloud_ts = 0.0
        self._target_locked = False
        self._last_attempt_signature = ""
        self._last_attempt_ts = 0.0
        self._last_publish_summary = ""

        self.create_subscription(
            PointCloud2,
            self.target_cloud_topic,
            self._on_target_cloud,
            qos_reliable,
        )
        if self.target_locked_topic:
            self.create_subscription(
                Bool,
                self.target_locked_topic,
                self._on_target_locked,
                qos_reliable,
            )
        self.topk_cloud_pub = self.create_publisher(PointCloud2, self.topk_cloud_topic, qos_latched)
        self.topk_contact_point_1_pub = self.create_publisher(
            PointCloud2, self.topk_contact_point_1_topic, qos_latched
        )
        self.topk_contact_point_2_pub = self.create_publisher(
            PointCloud2, self.topk_contact_point_2_topic, qos_latched
        )
        self.create_timer(1.0 / self.publish_rate_hz, self._maybe_publish_topk_cloud)

        self.get_logger().info(
            "graspgen_topk_bridge_node started: "
            f"target_cloud={self.target_cloud_topic}, topk_cloud={self.topk_cloud_topic}, "
            f"contact_point_1={self.topk_contact_point_1_topic}, "
            f"contact_point_2={self.topk_contact_point_2_topic}, "
            f"pair_markers={self.topk_pair_markers_topic}, server={self.graspgen_host}:{self.graspgen_port}, "
            f"require_target_locked={self.require_target_locked}"
        )

    def _on_target_cloud(self, msg: PointCloud2) -> None:
        self._latest_target_cloud = msg
        self._latest_target_cloud_ts = time.time()

    def _on_target_locked(self, msg: Bool) -> None:
        self._target_locked = bool(msg.data)

    def _maybe_publish_topk_cloud(self) -> None:
        if self.require_target_locked and not self._target_locked:
            return

        cloud_msg = self._latest_target_cloud
        if cloud_msg is None:
            return
        if self.sensor_stale_sec > 0.0 and (time.time() - self._latest_target_cloud_ts) > self.sensor_stale_sec:
            return

        signature = _stamp_to_signature(cloud_msg)
        now = time.time()
        if (
            signature == self._last_attempt_signature
            and (now - self._last_attempt_ts) < self.retry_same_cloud_sec
        ):
            return
        self._last_attempt_signature = signature
        self._last_attempt_ts = now

        points = _cloud_to_xyz_array(cloud_msg)
        if points.shape[0] < self.min_points_required:
            self.get_logger().warning(
                f"target cloud too small for GraspGen bridge: {points.shape[0]} < {self.min_points_required}"
            )
            if self.publish_empty_on_failure:
                self._publish_points(
                    np.empty((0, 3), dtype=np.float32),
                    frame_id=str(cloud_msg.header.frame_id or self.target_frame),
                    stamp=cloud_msg.header.stamp,
                )
                self._publish_contact_point_clouds(
                    np.empty((0, 3), dtype=np.float32),
                    np.empty((0, 3), dtype=np.float32),
                    frame_id=str(cloud_msg.header.frame_id or self.target_frame),
                    stamp=cloud_msg.header.stamp,
                )
            return

        if self.max_points > 0 and points.shape[0] > self.max_points:
            step = max(1, points.shape[0] // self.max_points)
            points = points[::step][: self.max_points]

        try:
            topk_geometry = self._query_graspgen(points)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"GraspGen top-k bridge failed: {exc}")
            if self.publish_empty_on_failure:
                self._publish_points(
                    np.empty((0, 3), dtype=np.float32),
                    frame_id=str(cloud_msg.header.frame_id or self.target_frame),
                    stamp=cloud_msg.header.stamp,
                )
                self._publish_contact_point_clouds(
                    np.empty((0, 3), dtype=np.float32),
                    np.empty((0, 3), dtype=np.float32),
                    frame_id=str(cloud_msg.header.frame_id or self.target_frame),
                    stamp=cloud_msg.header.stamp,
                )
            return

        frame_id = str(cloud_msg.header.frame_id or self.target_frame)
        contact_point_1 = np.asarray(topk_geometry["contact_point_1"], dtype=np.float32).reshape((-1, 3))
        contact_point_2 = np.asarray(topk_geometry["contact_point_2"], dtype=np.float32).reshape((-1, 3))
        topk_line_points = _build_pair_line_cloud(
            contact_point_1,
            contact_point_2,
            samples_per_pair=self.line_samples_per_pair,
        )
        self._publish_points(
            topk_line_points,
            frame_id=frame_id,
            stamp=cloud_msg.header.stamp,
        )
        self._publish_contact_point_clouds(
            contact_point_1,
            contact_point_2,
            frame_id=frame_id,
            stamp=cloud_msg.header.stamp,
        )
        pair_count = min(contact_point_1.shape[0], contact_point_2.shape[0])
        summary = (
            f"input_points={points.shape[0]} topk_pairs={pair_count} "
            f"line_points={topk_line_points.shape[0]}"
        )
        if summary != self._last_publish_summary:
            self.get_logger().info(f"published GraspGen top-k cloud: {summary}")
            self._last_publish_summary = summary

    def _publish_points(self, points: np.ndarray, *, frame_id: str, stamp) -> None:
        cloud = make_xyz_cloud(np.asarray(points, dtype=np.float32).reshape((-1, 3)), frame_id, stamp)
        self.topk_cloud_pub.publish(cloud)

    def _publish_contact_point_clouds(
        self,
        contact_point_1: np.ndarray,
        contact_point_2: np.ndarray,
        *,
        frame_id: str,
        stamp,
    ) -> None:
        points_1 = np.asarray(contact_point_1, dtype=np.float32).reshape((-1, 3))
        points_2 = np.asarray(contact_point_2, dtype=np.float32).reshape((-1, 3))
        c1_cloud = make_xyz_cloud(points_1, frame_id, stamp)
        c2_cloud = make_xyz_cloud(points_2, frame_id, stamp)
        self.topk_contact_point_1_pub.publish(c1_cloud)
        self.topk_contact_point_2_pub.publish(c2_cloud)

    def _query_graspgen(self, points: np.ndarray) -> dict[str, np.ndarray]:
        if not self.helper_script.is_file():
            raise FileNotFoundError(f"missing helper script: {self.helper_script}")
        graspgen_python = Path(self.graspgen_python).expanduser()
        if not graspgen_python.is_file():
            raise FileNotFoundError(f"missing GraspGen Python interpreter: {graspgen_python}")

        with tempfile.TemporaryDirectory(prefix="programme_graspgen_topk_") as temp_dir:
            temp_root = Path(temp_dir)
            cloud_path = temp_root / "target_cloud.npy"
            out_path = temp_root / "topk_geometry.npz"
            np.save(cloud_path, np.asarray(points, dtype=np.float32).reshape((-1, 3)))

            cmd = [
                str(graspgen_python),
                str(self.helper_script),
                "--repo-root",
                self.graspgen_repo_root,
                "--cloud-npy",
                str(cloud_path),
                "--out-npy",
                str(out_path),
                "--host",
                self.graspgen_host,
                "--port",
                str(self.graspgen_port),
                "--timeout-sec",
                str(self.request_timeout_sec),
                "--grasp-threshold",
                str(self.grasp_threshold),
                "--num-grasps",
                str(self.num_grasps),
                "--topk-num-grasps",
                str(self.topk_num_grasps),
                "--min-grasps",
                str(self.min_grasps),
                "--max-tries",
                str(self.max_tries),
                "--max-gripper-width-m",
                str(self.max_gripper_width_m),
            ]
            if self.remove_outliers:
                cmd.append("--remove-outliers")

            env = os.environ.copy()
            env.pop("PYTHONHOME", None)
            env.pop("PYTHONPATH", None)
            env["VIRTUAL_ENV"] = str(graspgen_python.parent.parent)
            env["PATH"] = f"{graspgen_python.parent}:{env.get('PATH', '')}"
            completed = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.subprocess_timeout_sec,
                check=False,
                env=env,
            )
            if completed.returncode != 0:
                detail = (completed.stderr or completed.stdout or "").strip()
                detail = detail[:600] if detail else f"exit code {completed.returncode}"
                raise RuntimeError(detail)
            if not out_path.is_file():
                raise RuntimeError("GraspGen helper did not produce top-k output")

            loaded = np.load(out_path)
            centers = np.asarray(loaded["centers"], dtype=np.float32).reshape((-1, 3))
            contact_point_1 = np.asarray(loaded["contact_point_1"], dtype=np.float32).reshape((-1, 3))
            contact_point_2 = np.asarray(loaded["contact_point_2"], dtype=np.float32).reshape((-1, 3))
            limit = max(
                0,
                min(
                    int(self.topk_num_grasps),
                    centers.shape[0],
                    contact_point_1.shape[0],
                    contact_point_2.shape[0],
                ),
            )
            return {
                "centers": centers[:limit],
                "contact_point_1": contact_point_1[:limit],
                "contact_point_2": contact_point_2[:limit],
            }


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspGenTopKBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
