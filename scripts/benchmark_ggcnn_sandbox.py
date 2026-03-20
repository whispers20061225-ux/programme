#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class GpuSample:
    ts: float
    util: float
    mem_used_mb: float
    mem_total_mb: float


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
                self._samples.append(GpuSample(time.time(), util, mem_used, mem_total))
            except Exception:
                pass
            self._stop.wait(self._interval_sec)


def summarize_gpu(samples: list[GpuSample]) -> dict[str, float]:
    if not samples:
        return {}
    utils = [sample.util for sample in samples]
    mems = [sample.mem_used_mb for sample in samples]
    return {
        "gpu_util_avg": float(np.mean(utils)),
        "gpu_util_peak": float(np.max(utils)),
        "gpu_mem_used_avg_mb": float(np.mean(mems)),
        "gpu_mem_used_peak_mb": float(np.max(mems)),
        "sample_count": float(len(samples)),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline GG-CNN sandbox benchmark")
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--checkpoint-path", required=True)
    parser.add_argument("--input-path", required=True)
    parser.add_argument("--model", default="ggcnn2", choices=["ggcnn", "ggcnn2"])
    parser.add_argument("--mask-id", type=int, default=1)
    parser.add_argument("--input-size", type=int, default=300)
    parser.add_argument("--bbox-margin", type=int, default=20)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--warmup-runs", type=int, default=1)
    parser.add_argument("--benchmark-runs", type=int, default=3)
    parser.add_argument("--gpu-index", type=int, default=0)
    parser.add_argument("--sample-interval-sec", type=float, default=0.2)
    parser.add_argument("--device", default="cuda:0")
    return parser.parse_args()


def ensure_imports(repo_root: Path) -> None:
    sys.path.insert(0, str(repo_root))


def load_sample(input_path: Path, mask_id: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    payload = np.load(input_path, allow_pickle=True)
    if input_path.suffix == ".npy" and hasattr(payload, "item"):
        payload = payload.item()

    if isinstance(payload, dict):
        depth = np.asarray(payload["depth"])
        seg = np.asarray(payload["seg"] if "seg" in payload else payload["segmap"])
        intrinsics = np.asarray(payload["K"])
    else:
        depth = np.asarray(payload["depth"])
        seg = np.asarray(payload["seg"] if "seg" in payload else payload["segmap"])
        intrinsics = np.asarray(payload["K"])

    if mask_id > 0:
        mask = (seg == mask_id)
    else:
        mask = seg > 0
    return depth.astype(np.float32), mask.astype(np.uint8), intrinsics.astype(np.float64)


def compute_roi(mask: np.ndarray, margin: int, shape: tuple[int, int]) -> tuple[int, int, int, int]:
    ys, xs = np.nonzero(mask)
    if ys.size == 0 or xs.size == 0:
        raise RuntimeError("mask is empty")
    y0 = max(int(ys.min()) - margin, 0)
    y1 = min(int(ys.max()) + margin + 1, shape[0])
    x0 = max(int(xs.min()) - margin, 0)
    x1 = min(int(xs.max()) + margin + 1, shape[1])
    return y0, y1, x0, x1


def preprocess_depth(depth: np.ndarray, mask: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, dict[str, int]]:
    from utils.dataset_processing.image import DepthImage

    y0, y1, x0, x1 = compute_roi(mask, args.bbox_margin, depth.shape)
    roi_depth = depth[y0:y1, x0:x1].copy()
    roi_mask = mask[y0:y1, x0:x1].copy()

    roi_depth[roi_mask == 0] = 0.0
    roi = DepthImage(roi_depth)
    roi.inpaint(missing_value=0)
    roi.resize((args.input_size, args.input_size))
    roi.normalise()

    meta = {
        "roi_y0": y0,
        "roi_y1": y1,
        "roi_x0": x0,
        "roi_x1": x1,
        "roi_h": y1 - y0,
        "roi_w": x1 - x0,
        "mask_pixels_full": int(mask.sum()),
        "mask_pixels_roi": int(roi_mask.sum()),
        "valid_depth_full": int((depth > 0).sum()),
        "valid_depth_masked": int(((depth > 0) & (mask > 0)).sum()),
    }
    return roi.img.astype(np.float32), meta


def load_model(repo_root: Path, model_name: str, checkpoint_path: Path, device: str):
    import torch

    if model_name == "ggcnn2":
        from models.ggcnn2 import GGCNN2 as Model
    else:
        from models.ggcnn import GGCNN as Model

    model = Model()
    state = torch.load(checkpoint_path, map_location=device)
    if isinstance(state, dict):
        try:
            model.load_state_dict(state)
        except Exception:
            maybe_model = state.get("model_state_dict")
            if maybe_model is None:
                raise
            model.load_state_dict(maybe_model)
    else:
        model = state
    model = model.to(device)
    model.eval()
    return model


def run_once(model, tensor, device: str, top_k: int):
    import torch
    from models.common import post_process_output
    from utils.dataset_processing.grasp import detect_grasps

    t0 = time.perf_counter()
    with torch.no_grad():
        pos, cos, sin, width = model(tensor)
        if device.startswith("cuda"):
            torch.cuda.synchronize()
    elapsed = time.perf_counter() - t0

    q_img, ang_img, width_img = post_process_output(pos, cos, sin, width)
    grasps = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=top_k)
    top_q = float(np.max(q_img)) if q_img.size else math.nan
    top_grasp = None
    if grasps:
        g = grasps[0]
        top_grasp = {
            "center_yx": [int(g.center[0]), int(g.center[1])],
            "angle_rad": float(g.angle),
            "length_px": float(g.length),
            "width_px": float(g.width),
        }
    return elapsed, len(grasps), top_q, top_grasp


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    checkpoint_path = Path(args.checkpoint_path).resolve()
    input_path = Path(args.input_path).resolve()

    ensure_imports(repo_root)

    import torch

    depth, mask, intrinsics = load_sample(input_path, args.mask_id)
    proc_depth, meta = preprocess_depth(depth, mask, args)

    device = args.device if torch.cuda.is_available() else "cpu"

    model_load_start = time.perf_counter()
    model = load_model(repo_root, args.model, checkpoint_path, device)
    model_load_sec = time.perf_counter() - model_load_start

    tensor = torch.from_numpy(proc_depth).unsqueeze(0).unsqueeze(0).to(device=device, dtype=torch.float32)

    warm_monitor = GpuMonitor(args.gpu_index, args.sample_interval_sec)
    warm_monitor.start()
    warm_results = [run_once(model, tensor, device, args.top_k) for _ in range(args.warmup_runs)]
    warm_gpu = summarize_gpu(warm_monitor.stop())

    bench_monitor = GpuMonitor(args.gpu_index, args.sample_interval_sec)
    bench_monitor.start()
    bench_results = [run_once(model, tensor, device, args.top_k) for _ in range(args.benchmark_runs)]
    bench_gpu = summarize_gpu(bench_monitor.stop())

    summary = {
        "repo_root": str(repo_root),
        "checkpoint_path": str(checkpoint_path),
        "input_path": str(input_path),
        "model": args.model,
        "device": device,
        "model_load_sec": model_load_sec,
        "preprocess": meta,
        "intrinsics": intrinsics.tolist(),
        "warmup": [
            {"elapsed_sec": elapsed, "candidate_count": count, "top_q": top_q, "top_grasp": top_grasp}
            for elapsed, count, top_q, top_grasp in warm_results
        ],
        "benchmark": [
            {"elapsed_sec": elapsed, "candidate_count": count, "top_q": top_q, "top_grasp": top_grasp}
            for elapsed, count, top_q, top_grasp in bench_results
        ],
        "warmup_gpu": warm_gpu,
        "benchmark_gpu": bench_gpu,
        "benchmark_elapsed_avg_sec": float(np.mean([item[0] for item in bench_results])) if bench_results else math.nan,
        "benchmark_elapsed_min_sec": float(np.min([item[0] for item in bench_results])) if bench_results else math.nan,
        "benchmark_elapsed_max_sec": float(np.max([item[0] for item in bench_results])) if bench_results else math.nan,
    }
    print(json.dumps(summary, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
