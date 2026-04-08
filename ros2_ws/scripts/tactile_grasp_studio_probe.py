#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Optional


BOUNDARY = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"


def jpeg_size(data: bytes) -> tuple[int, int]:
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        return 0, 0
    index = 2
    sof_markers = {
        0xC0,
        0xC1,
        0xC2,
        0xC3,
        0xC5,
        0xC6,
        0xC7,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
    }
    while index + 8 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        index += 2
        if marker in {0xD8, 0xD9}:
            continue
        if index + 2 > len(data):
            break
        segment_len = int.from_bytes(data[index : index + 2], "big")
        if segment_len < 2 or index + segment_len > len(data):
            break
        if marker in sof_markers and index + 7 < len(data):
            height = int.from_bytes(data[index + 3 : index + 5], "big")
            width = int.from_bytes(data[index + 5 : index + 7], "big")
            return width, height
        index += segment_len
    return 0, 0


@dataclass
class StreamProbeResult:
    url: str
    frame_count: int = 0
    changed_frame_count: int = 0
    first_frame_at: Optional[float] = None
    last_frame_at: Optional[float] = None
    max_gap_sec: float = 0.0
    avg_frame_bytes: float = 0.0
    width: int = 0
    height: int = 0
    open_error: str = ""
    end_reason: str = ""
    _total_bytes: int = 0
    _last_frame: Optional[bytes] = None

    def on_frame(self, frame: bytes) -> None:
        now = time.time()
        if self.first_frame_at is None:
            self.first_frame_at = now
        if self.last_frame_at is not None:
            self.max_gap_sec = max(self.max_gap_sec, now - self.last_frame_at)
        self.last_frame_at = now
        self.frame_count += 1
        self._total_bytes += len(frame)
        if self._last_frame != frame:
            self.changed_frame_count += 1
            self._last_frame = frame
        if self.width == 0 or self.height == 0:
            self.width, self.height = jpeg_size(frame)

    def finalize(self) -> dict[str, object]:
        active_span = 0.0
        if self.first_frame_at is not None and self.last_frame_at is not None:
            active_span = max(0.0, self.last_frame_at - self.first_frame_at)
        approx_fps = float(self.frame_count - 1) / active_span if self.frame_count >= 2 and active_span > 0 else 0.0
        approx_changed_fps = (
            float(self.changed_frame_count) / active_span if self.changed_frame_count >= 1 and active_span > 0 else 0.0
        )
        if self.frame_count:
            self.avg_frame_bytes = float(self._total_bytes) / float(self.frame_count)
        return {
            "url": self.url,
            "frame_count": self.frame_count,
            "changed_frame_count": self.changed_frame_count,
            "approx_fps": approx_fps,
            "approx_changed_fps": approx_changed_fps,
            "active_span_sec": active_span,
            "max_gap_sec": self.max_gap_sec,
            "avg_frame_bytes": self.avg_frame_bytes,
            "width": self.width,
            "height": self.height,
            "open_error": self.open_error,
            "end_reason": self.end_reason,
        }


def probe_stream(url: str, deadline: float, result: StreamProbeResult) -> None:
    try:
        request = urllib.request.Request(url, headers={"Cache-Control": "no-cache"})
        with urllib.request.urlopen(request, timeout=10) as response:
            buffer = b""
            while time.time() < deadline:
                chunk = response.read(8192)
                if not chunk:
                    result.end_reason = "eof"
                    return
                buffer += chunk
                while True:
                    first = buffer.find(BOUNDARY)
                    if first < 0:
                        if len(buffer) > len(BOUNDARY):
                            buffer = buffer[-len(BOUNDARY) :]
                        break
                    second = buffer.find(BOUNDARY, first + len(BOUNDARY))
                    if second < 0:
                        if first > 0:
                            buffer = buffer[first:]
                        break
                    frame = buffer[first + len(BOUNDARY) : second]
                    if frame.endswith(b"\r\n"):
                        frame = frame[:-2]
                    if frame:
                        result.on_frame(frame)
                    buffer = buffer[second:]
            result.end_reason = "deadline"
    except urllib.error.URLError as exc:
        result.open_error = str(exc)
    except Exception as exc:  # noqa: BLE001
        result.open_error = f"{type(exc).__name__}: {exc}"


@dataclass
class AvailabilityResult:
    url: str
    poll_interval_sec: float
    success_count: int = 0
    failure_count: int = 0
    first_failure_at: Optional[float] = None
    last_success_at: Optional[float] = None
    last_error: str = ""
    transitions: list[dict[str, object]] = field(default_factory=list)

    def record(self, ok: bool, detail: str) -> None:
        now = time.time()
        if ok:
            self.success_count += 1
            self.last_success_at = now
            current_state = "ok"
        else:
            self.failure_count += 1
            self.last_error = detail
            if self.first_failure_at is None:
                self.first_failure_at = now
            current_state = "error"
        if not self.transitions or self.transitions[-1]["state"] != current_state:
            self.transitions.append({"state": current_state, "at": now, "detail": detail})

    def finalize(self) -> dict[str, object]:
        return {
            "url": self.url,
            "poll_interval_sec": self.poll_interval_sec,
            "success_count": self.success_count,
            "failure_count": self.failure_count,
            "first_failure_at": self.first_failure_at,
            "last_success_at": self.last_success_at,
            "last_error": self.last_error,
            "transitions": self.transitions,
        }


def poll_url(url: str, deadline: float, result: AvailabilityResult) -> None:
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=5) as response:
                result.record(200 <= response.status < 300, f"http_{response.status}")
        except Exception as exc:  # noqa: BLE001
            result.record(False, f"{type(exc).__name__}: {exc}")
        time.sleep(result.poll_interval_sec)


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe Tactile Grasp Studio availability and stream rates.")
    parser.add_argument("--base-url", default="http://127.0.0.1:5173", help="Base URL for the web UI or gateway.")
    parser.add_argument("--duration", type=float, default=30.0, help="How long to sample, in seconds.")
    parser.add_argument(
        "--bootstrap-interval",
        type=float,
        default=2.0,
        help="Polling interval for /api/bootstrap availability checks.",
    )
    args = parser.parse_args()

    base_url = args.base_url.rstrip("/")
    deadline = time.time() + max(1.0, float(args.duration))

    availability = AvailabilityResult(
        url=f"{base_url}/api/bootstrap",
        poll_interval_sec=max(0.2, float(args.bootstrap_interval)),
    )
    availability_thread = threading.Thread(
        target=poll_url,
        args=(availability.url, deadline, availability),
        daemon=True,
    )

    streams = {
        "rgb": StreamProbeResult(url=f"{base_url}/api/streams/rgb.mjpeg"),
        "detection_overlay": StreamProbeResult(
            url=f"{base_url}/api/streams/detection_overlay.mjpeg"
        ),
        "grasp_overlay": StreamProbeResult(url=f"{base_url}/api/streams/grasp_overlay.mjpeg"),
    }

    threads = [
        threading.Thread(target=probe_stream, args=(name_result.url, deadline, name_result), daemon=True)
        for name_result in streams.values()
    ]

    availability_thread.start()
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    availability_thread.join()

    diagnostics_url = f"{base_url}/api/diagnostics/streams"
    diagnostics: dict[str, object]
    try:
        with urllib.request.urlopen(diagnostics_url, timeout=5) as response:
            diagnostics = json.loads(response.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001
        diagnostics = {"error": f"{type(exc).__name__}: {exc}", "url": diagnostics_url}

    output = {
        "base_url": base_url,
        "duration_sec": float(args.duration),
        "availability": availability.finalize(),
        "streams": {name: result.finalize() for name, result in streams.items()},
        "gateway_diagnostics": diagnostics,
    }
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
