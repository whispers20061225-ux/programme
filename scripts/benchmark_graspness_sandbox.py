#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class GpuSample:
    ts: float
    util: float
    mem_used_mb: float
    mem_total_mb: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline Graspness sandbox benchmark")
    parser.add_argument("--repo-root", required=True, help="Path to graspness_unofficial repo")
    parser.add_argument("--checkpoint-path", required=True, help="Path to model checkpoint")
    parser.add_argument("--input-path", required=True, help="Path to .npz or .npy depth sample")
    parser.add_argument("--camera", default="realsense", choices=["realsense", "kinect"])
    parser.add_argument("--num-point", type=int, default=15000)
    parser.add_argument("--voxel-size", type=float, default=0.005)
    parser.add_argument("--seed-feat-dim", type=int, default=512)
    parser.add_argument("--warmup-runs", type=int, default=1)
    parser.add_argument("--benchmark-runs", type=int, default=3)
    parser.add_argument("--top-k", type=int, default=50)
    parser.add_argument("--mask-id", type=int, default=1, help="Positive label inside segmap to keep")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--gpu-index", type=int, default=0)
    parser.add_argument("--sample-interval-sec", type=float, default=0.2)
    return parser.parse_args()


def ensure_repo_imports(repo_root: Path) -> None:
    sys.path.insert(0, str(repo_root))
    sys.path.insert(0, str(repo_root / "utils"))


def load_sample(input_path: Path, mask_id: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    if input_path.suffix == ".npz":
        payload = np.load(input_path, allow_pickle=True)
    elif input_path.suffix == ".npy":
        payload = np.load(input_path, allow_pickle=True)
        if hasattr(payload, "item"):
            payload = payload.item()
    else:
        raise ValueError(f"unsupported input type: {input_path}")

    if isinstance(payload, dict):
        depth = np.asarray(payload.get("depth"))
        segmap = np.asarray(payload.get("seg") if "seg" in payload else payload.get("segmap"))
        intrinsics = np.asarray(payload.get("K"))
        factor_depth = float(payload.get("factor_depth", 1000.0))
    else:
        depth = np.asarray(payload["depth"])
        segmap = np.asarray(payload["seg"] if "seg" in payload else payload["segmap"])
        intrinsics = np.asarray(payload["K"])
        factor_depth = float(payload["factor_depth"]) if "factor_depth" in payload else 1000.0

    if depth.ndim != 2:
        raise ValueError(f"depth must be HxW, got {depth.shape}")
    if segmap.shape != depth.shape:
        raise ValueError(f"segmap shape {segmap.shape} does not match depth {depth.shape}")
    if intrinsics.shape != (3, 3):
        raise ValueError(f"K must be 3x3, got {intrinsics.shape}")

    if mask_id > 0:
        segmap = (segmap == mask_id).astype(np.uint8)
    else:
        segmap = (segmap > 0).astype(np.uint8)
    return depth, segmap, intrinsics, factor_depth


def build_input_dict(
    depth: np.ndarray,
    segmap: np.ndarray,
    intrinsics: np.ndarray,
    factor_depth: float,
    num_point: int,
    voxel_size: float,
) -> dict[str, np.ndarray]:
    from data_utils import CameraInfo, create_point_cloud_from_depth_image

    camera = CameraInfo(
        float(depth.shape[1]),
        float(depth.shape[0]),
        float(intrinsics[0, 0]),
        float(intrinsics[1, 1]),
        float(intrinsics[0, 2]),
        float(intrinsics[1, 2]),
        float(factor_depth),
    )
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
    valid_mask = (depth > 0) & (segmap > 0)
    cloud_masked = cloud[valid_mask]
    if cloud_masked.size == 0:
        raise RuntimeError("no valid masked depth pixels after preprocessing")

    if len(cloud_masked) >= num_point:
        idxs = np.random.choice(len(cloud_masked), num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), num_point - len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)

    cloud_sampled = cloud_masked[idxs].astype(np.float32)
    return {
        "point_clouds": cloud_sampled,
        "coors": (cloud_sampled / voxel_size).astype(np.float32),
        "feats": np.ones_like(cloud_sampled, dtype=np.float32),
        "raw_point_count": np.array([len(cloud_masked)], dtype=np.int32),
        "masked_depth_pixels": np.array([int(valid_mask.sum())], dtype=np.int32),
    }


class GpuMonitor:
    def __init__(self, gpu_index: int, interval_sec: float) -> None:
        self._gpu_index = gpu_index
        self._interval_sec = interval_sec
        self._samples: list[GpuSample] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> list[GpuSample]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        return list(self._samples)

    def _run(self) -> None:
        cmd = [
            "nvidia-smi",
            f"--id={self._gpu_index}",
            "--query-gpu=utilization.gpu,memory.used,memory.total",
            "--format=csv,noheader,nounits",
        ]
        while not self._stop.is_set():
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0, check=True)
                line = result.stdout.strip().splitlines()[0]
                util, mem_used, mem_total = [float(x.strip()) for x in line.split(",")]
                self._samples.append(
                    GpuSample(
                        ts=time.time(),
                        util=util,
                        mem_used_mb=mem_used,
                        mem_total_mb=mem_total,
                    )
                )
            except Exception:
                pass
            self._stop.wait(self._interval_sec)


def move_batch_to_device(batch: dict[str, Any], device: Any) -> dict[str, Any]:
    import torch

    moved: dict[str, Any] = {}
    for key, value in batch.items():
        if isinstance(value, list):
            moved[key] = []
            for item in value:
                if isinstance(item, list):
                    moved[key].append([sub_item.to(device) for sub_item in item])
                else:
                    moved[key].append(item.to(device))
        elif hasattr(value, "to"):
            moved[key] = value.to(device)
        else:
            moved[key] = value
    return moved


def summarize_gpu(samples: list[GpuSample]) -> dict[str, float]:
    if not samples:
        return {}
    utils = [sample.util for sample in samples]
    mems = [sample.mem_used_mb for sample in samples]
    return {
        "gpu_util_avg": float(np.mean(utils)),
        "gpu_util_peak": float(np.max(utils)),
        "gpu_mem_used_peak_mb": float(np.max(mems)),
        "gpu_mem_used_avg_mb": float(np.mean(mems)),
        "sample_count": float(len(samples)),
    }


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    checkpoint_path = Path(args.checkpoint_path).resolve()
    input_path = Path(args.input_path).resolve()

    ensure_repo_imports(repo_root)

    import torch
    from dataset.graspnet_dataset import minkowski_collate_fn
    from graspnetAPI.grasp import GraspGroup
    from models.graspnet import GraspNet, pred_decode

    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint_path}")

    depth, segmap, intrinsics, factor_depth = load_sample(input_path, args.mask_id)
    data_input = build_input_dict(
        depth=depth,
        segmap=segmap,
        intrinsics=intrinsics,
        factor_depth=factor_depth,
        num_point=args.num_point,
        voxel_size=args.voxel_size,
    )

    batch = minkowski_collate_fn([data_input])
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")

    model_start = time.perf_counter()
    net = GraspNet(seed_feat_dim=args.seed_feat_dim, is_training=False).to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    state_dict = checkpoint["model_state_dict"] if "model_state_dict" in checkpoint else checkpoint
    net.load_state_dict(state_dict)
    net.eval()
    model_load_sec = time.perf_counter() - model_start

    def run_once() -> tuple[float, int, float]:
        local_batch = move_batch_to_device(batch, device)
        t0 = time.perf_counter()
        with torch.no_grad():
            end_points = net(local_batch)
            grasp_preds = pred_decode(end_points)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
        elapsed = time.perf_counter() - t0
        preds = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(preds)
        gg = gg.sort_by_score()
        candidate_count = len(gg)
        top_score = float(gg[0].score) if candidate_count else math.nan
        return elapsed, candidate_count, top_score

    warm_monitor = GpuMonitor(args.gpu_index, args.sample_interval_sec)
    warm_monitor.start()
    warm_results = [run_once() for _ in range(args.warmup_runs)]
    warm_gpu = warm_monitor.stop()

    bench_monitor = GpuMonitor(args.gpu_index, args.sample_interval_sec)
    bench_monitor.start()
    bench_results = [run_once() for _ in range(args.benchmark_runs)]
    bench_gpu = bench_monitor.stop()

    warm_times = [item[0] for item in warm_results]
    hot_times = [item[0] for item in bench_results]
    candidate_counts = [item[1] for item in bench_results]
    top_scores = [item[2] for item in bench_results]

    summary = {
        "repo_root": str(repo_root),
        "checkpoint_path": str(checkpoint_path),
        "input_path": str(input_path),
        "device": str(device),
        "torch_cuda_available": bool(torch.cuda.is_available()),
        "cuda_device_name": torch.cuda.get_device_name(device) if device.type == "cuda" else "",
        "depth_shape": [int(depth.shape[0]), int(depth.shape[1])],
        "mask_pixels": int(segmap.sum()),
        "raw_point_count": int(data_input["raw_point_count"][0]),
        "sampled_point_count": int(data_input["point_clouds"].shape[0]),
        "model_load_sec": model_load_sec,
        "warmup_runs": args.warmup_runs,
        "benchmark_runs": args.benchmark_runs,
        "warmup_sec": warm_times,
        "hot_sec": hot_times,
        "hot_sec_mean": float(np.mean(hot_times)),
        "hot_sec_p95": float(np.percentile(hot_times, 95)),
        "candidate_count_mean": float(np.mean(candidate_counts)),
        "candidate_count_max": int(np.max(candidate_counts)) if candidate_counts else 0,
        "top_score_mean": float(np.nanmean(top_scores)) if top_scores else math.nan,
        "warm_gpu": summarize_gpu(warm_gpu),
        "hot_gpu": summarize_gpu(bench_gpu),
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
