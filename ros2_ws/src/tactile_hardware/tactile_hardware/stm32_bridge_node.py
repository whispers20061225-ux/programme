from __future__ import annotations

import math
import socket
import time
from collections import deque
from typing import Deque, List, Optional, Tuple
from urllib.parse import urlparse

try:
    import serial
except ImportError:
    serial = None

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger

from tactile_interfaces.msg import SystemHealth, TactileRaw

from .stm32_tactile_codec import ParsedFrame, decode_taxels, decode_totals, parse_reply_line


class TcpLineClient:
    def __init__(self, host: str, port: int, timeout: float) -> None:
        self._timeout = max(0.05, float(timeout))
        self._socket = socket.create_connection((host, port), timeout=self._timeout)
        self._socket.settimeout(self._timeout)
        self._buffer = bytearray()

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    def write(self, data: bytes) -> None:
        if self._socket is None:
            raise RuntimeError("tcp socket is closed")
        self._socket.sendall(data)

    def flush(self) -> None:
        return

    def readline(self) -> bytes:
        if self._socket is None:
            return b""
        deadline = time.time() + self._timeout
        while time.time() < deadline:
            newline_index = self._buffer.find(b"\n")
            if newline_index >= 0:
                line = bytes(self._buffer[: newline_index + 1])
                del self._buffer[: newline_index + 1]
                return line
            remaining = max(0.01, deadline - time.time())
            self._socket.settimeout(remaining)
            chunk = self._socket.recv(4096)
            if not chunk:
                raise RuntimeError("tcp bridge closed the connection")
            self._buffer.extend(chunk)
        return b""

    def reset_input_buffer(self) -> None:
        self._buffer.clear()
        if self._socket is None:
            return
        try:
            self._socket.setblocking(False)
            while True:
                chunk = self._socket.recv(4096)
                if not chunk:
                    break
        except (BlockingIOError, TimeoutError):
            pass
        finally:
            self._socket.setblocking(True)
            self._socket.settimeout(self._timeout)

    def reset_output_buffer(self) -> None:
        return

    def close(self) -> None:
        if self._socket is None:
            return
        try:
            self._socket.close()
        finally:
            self._socket = None


class VectorProcessor:
    def __init__(
        self,
        *,
        baseline_enabled: bool,
        baseline_frames: int,
        median_window: int,
        ema_alpha: float,
        clip_negative: bool,
    ) -> None:
        self._baseline_enabled = bool(baseline_enabled)
        self._baseline_frames = max(1, int(baseline_frames))
        self._median_window = max(1, int(median_window))
        self._ema_alpha = max(0.0, min(1.0, float(ema_alpha)))
        self._clip_negative = bool(clip_negative)
        self.reset()

    @property
    def baseline_ready(self) -> bool:
        return (not self._baseline_enabled) or self._baseline is not None

    def reset(self) -> None:
        self._value_count = 0
        self._history: Deque[List[float]] = deque(maxlen=self._median_window)
        self._ema_values: Optional[List[float]] = None
        self._baseline_sum: Optional[List[float]] = None
        self._baseline: Optional[List[float]] = None
        self._baseline_count = 0

    def process(self, values: List[float]) -> Tuple[List[float], List[str]]:
        if not values:
            self.reset()
            return [], []

        if self._value_count != len(values):
            self.reset()
            self._value_count = len(values)

        processed = [float(item) for item in values]
        notes: List[str] = []

        if self._baseline_enabled:
            if self._baseline_sum is None:
                self._baseline_sum = [0.0] * len(processed)
            if self._baseline is None:
                self._baseline_sum = [
                    current + value for current, value in zip(self._baseline_sum, processed)
                ]
                self._baseline_count += 1
                notes.append(f"baseline {self._baseline_count}/{self._baseline_frames}")
                if self._baseline_count >= self._baseline_frames:
                    self._baseline = [
                        value / float(self._baseline_count) for value in self._baseline_sum
                    ]
                    notes.append("baseline_ready")
            if self._baseline is not None and len(self._baseline) == len(processed):
                processed = [value - base for value, base in zip(processed, self._baseline)]

        if self._clip_negative:
            processed = [max(0.0, value) for value in processed]

        if self._median_window > 1:
            self._history.append(list(processed))
            processed = self._median_reduce(self._history)
        else:
            self._history.clear()

        if self._ema_alpha > 0.0:
            if self._ema_values is None or len(self._ema_values) != len(processed):
                self._ema_values = list(processed)
            else:
                self._ema_values = [
                    (self._ema_alpha * value) + ((1.0 - self._ema_alpha) * previous)
                    for value, previous in zip(processed, self._ema_values)
                ]
            processed = list(self._ema_values)

        return processed, notes

    def _median_reduce(self, history: Deque[List[float]]) -> List[float]:
        if not history:
            return []
        value_count = len(history[0])
        result: List[float] = []
        for index in range(value_count):
            samples = sorted(frame[index] for frame in history if len(frame) == value_count)
            if not samples:
                result.append(0.0)
                continue
            middle = len(samples) // 2
            if len(samples) % 2 == 1:
                result.append(samples[middle])
            else:
                result.append((samples[middle - 1] + samples[middle]) / 2.0)
        return result


class Stm32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("stm32_bridge_node")

        self.declare_parameter("publish_rate_hz", 25.0)
        self.declare_parameter("requested_mode", "simulation")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        self.declare_parameter("serial_startup_delay_sec", 1.0)
        self.declare_parameter("command_timeout_sec", 1.2)
        self.declare_parameter("device_addr", 1)
        self.declare_parameter("sensor_layout", "m2020_3x3")
        self.declare_parameter("totals_command_template", "TREADX {device_addr} 1008 3")
        self.declare_parameter("taxels_command_template", "TREADX {device_addr} 1038 27")
        self.declare_parameter("grid_rows", 3)
        self.declare_parameter("grid_cols", 3)
        self.declare_parameter("baseline_enabled", True)
        self.declare_parameter("baseline_frames", 16)
        self.declare_parameter("median_window", 3)
        self.declare_parameter("ema_alpha", 0.30)
        self.declare_parameter("clip_negative_heatmap", True)
        self.declare_parameter("contact_threshold", 3.0)
        self.declare_parameter("contact_debounce_frames", 2)
        self.declare_parameter("simulation_rows", 3)
        self.declare_parameter("simulation_cols", 3)
        self.declare_parameter("tactile_topic", "/tactile/raw")
        self.declare_parameter("health_topic", "/system/health")
        self.declare_parameter("mode_service", "/tactile/use_hardware")
        self.declare_parameter("tare_service", "/tactile/tare")
        self.declare_parameter("clear_tare_service", "/tactile/clear_tare")
        self.declare_parameter("reconnect_interval_sec", 2.0)

        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.requested_mode = self._normalize_mode(
            str(self.get_parameter("requested_mode").value or "")
        )
        self.serial_port = str(self.get_parameter("serial_port").value or "").strip()
        self.serial_baudrate = int(self.get_parameter("serial_baudrate").value)
        self.serial_timeout = max(0.05, float(self.get_parameter("serial_timeout").value))
        self.serial_startup_delay_sec = max(
            0.0, float(self.get_parameter("serial_startup_delay_sec").value)
        )
        self.command_timeout_sec = max(
            0.05, float(self.get_parameter("command_timeout_sec").value)
        )
        self.device_addr = max(1, int(self.get_parameter("device_addr").value))
        self.sensor_layout = str(self.get_parameter("sensor_layout").value or "m2020_3x3").strip()
        self.totals_command_template = str(
            self.get_parameter("totals_command_template").value or "TREADX {device_addr} 1008 3"
        )
        self.taxels_command_template = str(
            self.get_parameter("taxels_command_template").value or "TREADX {device_addr} 1038 27"
        )
        self.grid_rows = max(1, int(self.get_parameter("grid_rows").value))
        self.grid_cols = max(1, int(self.get_parameter("grid_cols").value))
        self.baseline_enabled = bool(self.get_parameter("baseline_enabled").value)
        self.baseline_frames = max(1, int(self.get_parameter("baseline_frames").value))
        self.median_window = max(1, int(self.get_parameter("median_window").value))
        self.ema_alpha = max(0.0, min(1.0, float(self.get_parameter("ema_alpha").value)))
        self.clip_negative_heatmap = bool(self.get_parameter("clip_negative_heatmap").value)
        self.contact_threshold = max(0.0, float(self.get_parameter("contact_threshold").value))
        self.contact_debounce_frames = max(
            1, int(self.get_parameter("contact_debounce_frames").value)
        )
        self.simulation_rows = max(1, int(self.get_parameter("simulation_rows").value))
        self.simulation_cols = max(1, int(self.get_parameter("simulation_cols").value))
        self.tactile_topic = str(self.get_parameter("tactile_topic").value)
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.mode_service = str(self.get_parameter("mode_service").value)
        self.tare_service = str(self.get_parameter("tare_service").value)
        self.clear_tare_service = str(self.get_parameter("clear_tare_service").value)
        self.reconnect_interval_sec = max(
            0.25, float(self.get_parameter("reconnect_interval_sec").value)
        )

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_health = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.tactile_pub = self.create_publisher(TactileRaw, self.tactile_topic, qos_sensor)
        self.health_pub = self.create_publisher(SystemHealth, self.health_topic, qos_health)
        self.create_service(SetBool, self.mode_service, self._on_set_mode)
        self.create_service(Trigger, self.tare_service, self._on_tare)
        self.create_service(Trigger, self.clear_tare_service, self._on_clear_tare)

        self._serial = None
        self._last_connect_try = 0.0
        self._sequence_id = 0
        self._phase = 0.0
        self._last_transport_stamp = 0.0
        self._transport_rate_hz = 0.0
        self._last_good_frame_stamp = 0.0
        self._connect_error = ""
        self._last_parser_state = "startup"
        self._last_mapping_state = "idle"
        self._last_status_text = "bridge starting"
        self._contact_active = False
        self._contact_hits = 0
        self._contact_misses = 0
        self._tare_active = False
        self._tare_updated_at = 0.0
        self._tare_total_offset: List[float] = [0.0, 0.0, 0.0]
        self._tare_fx_offset: List[float] = []
        self._tare_fy_offset: List[float] = []
        self._tare_fz_offset: List[float] = []
        self._last_output_updated_at = 0.0
        self._last_output_active_mode = "idle"
        self._last_output_source_connected = False
        self._last_output_total = [0.0, 0.0, 0.0]
        self._last_output_fx: List[float] = []
        self._last_output_fy: List[float] = []
        self._last_output_fz: List[float] = []

        self._totals_processor = VectorProcessor(
            baseline_enabled=self.baseline_enabled,
            baseline_frames=self.baseline_frames,
            median_window=self.median_window,
            ema_alpha=self.ema_alpha,
            clip_negative=False,
        )
        self._taxel_fx_processor = VectorProcessor(
            baseline_enabled=self.baseline_enabled,
            baseline_frames=self.baseline_frames,
            median_window=self.median_window,
            ema_alpha=self.ema_alpha,
            clip_negative=False,
        )
        self._taxel_fy_processor = VectorProcessor(
            baseline_enabled=self.baseline_enabled,
            baseline_frames=self.baseline_frames,
            median_window=self.median_window,
            ema_alpha=self.ema_alpha,
            clip_negative=False,
        )
        self._taxel_fz_processor = VectorProcessor(
            baseline_enabled=self.baseline_enabled,
            baseline_frames=self.baseline_frames,
            median_window=self.median_window,
            ema_alpha=self.ema_alpha,
            clip_negative=self.clip_negative_heatmap,
        )

        tick_period = max(0.01, 1.0 / self.publish_rate_hz)
        self.create_timer(tick_period, self._tick)
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            "stm32_bridge_node started: "
            f"requested_mode={self.requested_mode}, "
            f"serial_port={self.serial_port or '<unset>'}, "
            f"layout={self.sensor_layout}"
        )

    def _normalize_mode(self, value: str) -> str:
        normalized = str(value or "").strip().lower()
        if normalized in {"hardware", "real", "stm32"}:
            return "hardware"
        return "simulation"

    def _on_set_mode(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.requested_mode = "hardware" if bool(request.data) else "simulation"
        self._reset_processing_state()
        if self.requested_mode == "simulation":
            self._disconnect_serial()
        else:
            self._last_connect_try = 0.0
        self._last_parser_state = "mode_changed"
        self._last_mapping_state = "idle"
        self._last_status_text = f"requested_mode={self.requested_mode}"
        response.success = True
        response.message = self._last_status_text
        self.get_logger().info(response.message)
        return response

    def _on_tare(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if not self._has_recent_output():
            response.success = False
            response.message = "tare unavailable: no recent tactile frame"
            return response

        self._tare_total_offset = list(self._last_output_total)
        self._tare_fx_offset = list(self._last_output_fx)
        self._tare_fy_offset = list(self._last_output_fy)
        self._tare_fz_offset = list(self._last_output_fz)
        self._tare_active = True
        self._tare_updated_at = time.time()
        self._contact_active = False
        self._contact_hits = 0
        self._contact_misses = 0
        response.success = True
        response.message = (
            f"tare captured mode={self._last_output_active_mode} "
            f"cells={len(self._last_output_fz)} total_fz={self._last_output_total[2]:.2f}"
        )
        self.get_logger().info(response.message)
        return response

    def _on_clear_tare(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        self._clear_tare_state()
        response.success = True
        response.message = "tare cleared"
        self.get_logger().info(response.message)
        return response

    def _tick(self) -> None:
        if self.requested_mode == "simulation":
            self._publish_simulation()
            return

        if not self._ensure_connected():
            self._publish_empty(
                active_mode="idle",
                source_connected=False,
                parser_state="serial_disconnected",
                mapping_state="idle",
                status_text=self._connect_error or "hardware mode requested but serial is unavailable",
            )
            return

        totals_frame = self._request_frame(self.totals_command_template)
        taxels_frame = self._request_frame(self.taxels_command_template)

        totals_valid, totals_issue = self._validate_frame(
            totals_frame, expected_start_addr=1008, min_payload_len=3
        )
        taxels_valid, taxels_issue = self._validate_frame(
            taxels_frame, expected_start_addr=1038, min_payload_len=27
        )

        total_fx = 0.0
        total_fy = 0.0
        total_fz = 0.0
        forces_fx: List[float] = []
        forces_fy: List[float] = []
        forces_fz: List[float] = []
        heatmap: List[float] = []

        notes: List[str] = []
        if totals_issue:
            notes.append(f"totals={totals_issue}")
        if taxels_issue:
            notes.append(f"taxels={taxels_issue}")

        if totals_valid:
            totals = decode_totals(totals_frame.payload[:3])
            processed_totals, total_notes = self._totals_processor.process(
                [float(totals.fx), float(totals.fy), float(totals.fz)]
            )
            total_fx, total_fy, total_fz = processed_totals
            notes.extend(f"totals_{item}" for item in total_notes)
        else:
            self._totals_processor.reset()

        if taxels_valid:
            taxels = decode_taxels(taxels_frame.payload[:27])
            raw_fx = [float(item.fx) for item in taxels]
            raw_fy = [float(item.fy) for item in taxels]
            raw_fz = [float(item.fz) for item in taxels]
            forces_fx, fx_notes = self._taxel_fx_processor.process(raw_fx)
            forces_fy, fy_notes = self._taxel_fy_processor.process(raw_fy)
            forces_fz, fz_notes = self._taxel_fz_processor.process(raw_fz)
            heatmap = [max(0.0, value) for value in forces_fz]
            notes.extend(f"taxel_fx_{item}" for item in fx_notes)
            notes.extend(f"taxel_fy_{item}" for item in fy_notes)
            notes.extend(f"taxel_fz_{item}" for item in fz_notes)
        else:
            self._taxel_fx_processor.reset()
            self._taxel_fy_processor.reset()
            self._taxel_fz_processor.reset()

        total_fx, total_fy, total_fz = self._apply_tare_totals(total_fx, total_fy, total_fz)
        forces_fx = self._apply_tare_offsets(forces_fx, self._tare_fx_offset)
        forces_fy = self._apply_tare_offsets(forces_fy, self._tare_fy_offset)
        forces_fz = self._apply_tare_offsets(forces_fz, self._tare_fz_offset)
        heatmap = [max(0.0, value) for value in forces_fz]

        baseline_ready = self._baseline_ready()
        contact_score = max(0.0, float(total_fz))
        contact_active = self._update_contact_state(contact_score)
        frame_for_diag = taxels_frame if taxels_frame.raw else totals_frame
        parser_state = self._combine_parser_state(totals_valid, taxels_valid)
        mapping_state = "m2020_totals_taxels" if (totals_valid or taxels_valid) else "decode_unavailable"

        if not totals_valid and not taxels_valid:
            self._publish_empty(
                active_mode="hardware",
                source_connected=True,
                parser_state=parser_state,
                mapping_state="decode_unavailable",
                status_text="; ".join(notes) or "no valid tactile frame",
                frame=frame_for_diag if frame_for_diag.raw else None,
            )
            return

        if frame_for_diag.raw:
            if self._last_transport_stamp > 0.0:
                delta = time.time() - self._last_transport_stamp
                if delta > 0.0:
                    self._transport_rate_hz = 1.0 / delta
            self._last_transport_stamp = time.time()
            self._last_good_frame_stamp = self._last_transport_stamp

        raw_frame_hex = self._compose_raw_frame_hex(totals_frame, taxels_frame)
        status_text = "; ".join(notes) if notes else "m2020 tactile frame decoded"
        if self._tare_active:
            status_text = f"{status_text}; tare_active"

        self._publish_message(
            rows=self.grid_rows if forces_fz else 0,
            cols=self.grid_cols if forces_fz else 0,
            forces=heatmap,
            forces_fx=forces_fx,
            forces_fy=forces_fy,
            forces_fz=forces_fz,
            total_fx=total_fx,
            total_fy=total_fy,
            total_fz=total_fz,
            contact_active=contact_active,
            contact_score=contact_score,
            baseline_ready=baseline_ready,
            active_mode="hardware",
            source_connected=True,
            source_name=self.serial_port or "stm32-cdc",
            frame=frame_for_diag if frame_for_diag.raw else None,
            parser_state=parser_state,
            mapping_state=mapping_state,
            status_text=status_text,
            raw_frame_hex=raw_frame_hex,
        )

    def _request_frame(self, template: str) -> ParsedFrame:
        command = str(template).format(device_addr=self.device_addr)
        line = self._send_line_command(command, timeout=self.command_timeout_sec)
        if not line:
            return ParsedFrame(
                source=command.split()[0],
                raw=b"",
                raw_frame_hex="",
                observed_len=0,
                expected_total_len=0,
                frame_len_declared=0,
                device_addr=self.device_addr,
                function_code=0,
                start_addr=0,
                returned_data_len_hint=0,
                status=0,
                payload=b"",
                lrc_ok=False,
                parser_state="no_response",
                status_text=f"no response to {command}",
            )
        return parse_reply_line(line)

    def _validate_frame(
        self, frame: ParsedFrame, *, expected_start_addr: int, min_payload_len: int
    ) -> Tuple[bool, str]:
        if not frame.raw:
            return False, frame.status_text or frame.parser_state
        if frame.function_code != 0xFB:
            return False, f"unexpected_func=0x{frame.function_code:02X}"
        if frame.start_addr != expected_start_addr:
            return False, f"unexpected_addr={frame.start_addr}"
        if len(frame.payload) < min_payload_len:
            return False, f"payload_too_short={len(frame.payload)}"
        if not frame.lrc_ok:
            return False, "bad_lrc"
        if frame.status != 0:
            return False, f"device_status={frame.status}"
        return True, ""

    def _combine_parser_state(self, totals_valid: bool, taxels_valid: bool) -> str:
        if totals_valid and taxels_valid:
            return "totals_ok_taxels_ok"
        if totals_valid and not taxels_valid:
            return "totals_ok_taxels_fail"
        if taxels_valid and not totals_valid:
            return "totals_fail_taxels_ok"
        return "totals_fail_taxels_fail"

    def _compose_raw_frame_hex(self, totals_frame: ParsedFrame, taxels_frame: ParsedFrame) -> str:
        parts: List[str] = []
        if totals_frame.raw_frame_hex:
            parts.append(f"1008:{totals_frame.raw_frame_hex}")
        if taxels_frame.raw_frame_hex:
            parts.append(f"1038:{taxels_frame.raw_frame_hex}")
        return " | ".join(parts)

    def _baseline_ready(self) -> bool:
        return (
            self._totals_processor.baseline_ready
            and self._taxel_fx_processor.baseline_ready
            and self._taxel_fy_processor.baseline_ready
            and self._taxel_fz_processor.baseline_ready
        )

    def _update_contact_state(self, contact_score: float) -> bool:
        if contact_score >= self.contact_threshold:
            self._contact_hits += 1
            self._contact_misses = 0
        else:
            self._contact_misses += 1
            self._contact_hits = 0

        if not self._contact_active and self._contact_hits >= self.contact_debounce_frames:
            self._contact_active = True
        elif self._contact_active and self._contact_misses >= self.contact_debounce_frames:
            self._contact_active = False
        return self._contact_active

    def _reset_processing_state(self) -> None:
        self._totals_processor.reset()
        self._taxel_fx_processor.reset()
        self._taxel_fy_processor.reset()
        self._taxel_fz_processor.reset()
        self._contact_active = False
        self._contact_hits = 0
        self._contact_misses = 0
        self._clear_tare_state()

    def _is_tcp_endpoint(self) -> bool:
        return str(self.serial_port or "").strip().lower().startswith("tcp://")

    def _parse_tcp_endpoint(self) -> Tuple[str, int]:
        parsed = urlparse(self.serial_port)
        host = str(parsed.hostname or "127.0.0.1").strip() or "127.0.0.1"
        if host.lower() in {"windows-host", "wsl-host", "auto"}:
            host = self._resolve_windows_host()
        port = int(parsed.port or 0)
        if port <= 0:
            raise ValueError(f"invalid tcp endpoint: {self.serial_port}")
        return host, port

    def _resolve_windows_host(self) -> str:
        try:
            with open("/proc/net/route", "r", encoding="utf-8") as route_file:
                next(route_file, None)
                for line in route_file:
                    fields = line.strip().split()
                    if len(fields) < 4 or fields[1] != "00000000":
                        continue
                    flags = int(fields[3], 16)
                    if (flags & 0x2) == 0:
                        continue
                    gateway_value = int(fields[2], 16)
                    gateway_ip = socket.inet_ntoa(
                        gateway_value.to_bytes(4, byteorder="little", signed=False)
                    )
                    if gateway_ip:
                        return gateway_ip
        except Exception as exc:
            raise RuntimeError(f"failed to resolve Windows host gateway: {exc}") from exc
        raise RuntimeError("failed to resolve Windows host gateway")

    def _ensure_connected(self) -> bool:
        if self._serial is not None and getattr(self._serial, "is_open", False):
            return True
        if not self.serial_port:
            self._connect_error = "serial_port is empty"
            return False

        now = time.time()
        if now - self._last_connect_try < self.reconnect_interval_sec:
            return False
        self._last_connect_try = now

        try:
            if self._is_tcp_endpoint():
                host, port = self._parse_tcp_endpoint()
                ser = TcpLineClient(host, port, timeout=self.serial_timeout)
            else:
                if serial is None:
                    self._connect_error = "pyserial is not installed"
                    return False
                ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.serial_baudrate,
                    timeout=self.serial_timeout,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )
            self._serial = ser
            if self.serial_startup_delay_sec > 0.0:
                time.sleep(self.serial_startup_delay_sec)
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                pass
            pong = self._send_line_command("PING", timeout=self.command_timeout_sec)
            if not pong or "PONG" not in pong:
                raise RuntimeError(f"PING failed: {pong or 'no response'}")
            self._connect_error = ""
            transport_name = "tcp bridge" if self._is_tcp_endpoint() else "serial"
            self._last_status_text = f"connected on {self.serial_port} ({transport_name})"
            self.get_logger().info(self._last_status_text)
            return True
        except Exception as exc:
            self._connect_error = f"connect failed: {exc}"
            self._disconnect_serial()
            return False

    def _disconnect_serial(self) -> None:
        if self._serial is not None:
            try:
                if getattr(self._serial, "is_open", False):
                    self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _send_line_command(self, command: str, timeout: float) -> Optional[str]:
        if self._serial is None or not getattr(self._serial, "is_open", False):
            return None
        try:
            try:
                self._serial.reset_input_buffer()
            except Exception:
                pass
            payload = command.strip()
            if not payload.endswith("\n"):
                payload = f"{payload}\n"
            self._serial.write(payload.encode("utf-8"))
            self._serial.flush()

            deadline = time.time() + timeout
            while time.time() < deadline:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if line:
                    return line
            return None
        except Exception as exc:
            self._connect_error = f"transport command failed: {exc}"
            self._disconnect_serial()
            return None

    def _publish_simulation(self) -> None:
        rows = max(1, self.simulation_rows)
        cols = max(1, self.simulation_cols)
        fx: List[float] = []
        fy: List[float] = []
        fz: List[float] = []
        center_x = (cols - 1) / 2.0 + 0.45 * math.sin(self._phase * 0.8)
        center_y = (rows - 1) / 2.0 + 0.35 * math.cos(self._phase * 0.6)
        for index in range(rows * cols):
            row = index // cols
            col = index % cols
            dx = float(col) - center_x
            dy = float(row) - center_y
            radial = math.sqrt((dx * dx) + (dy * dy))
            weight = math.exp(-0.8 * radial * radial)
            fz_value = max(0.0, 2.0 + (16.0 * weight))
            fx_value = -2.8 * dx * weight
            fy_value = -2.8 * dy * weight
            fx.append(fx_value)
            fy.append(fy_value)
            fz.append(fz_value)

        total_fx = sum(fx)
        total_fy = sum(fy)
        total_fz = sum(fz)
        contact_score = max(0.0, total_fz / max(1.0, float(rows * cols)))
        self._contact_active = contact_score >= self.contact_threshold
        self._publish_message(
            rows=rows,
            cols=cols,
            forces=[max(0.0, item) for item in fz],
            forces_fx=fx,
            forces_fy=fy,
            forces_fz=fz,
            total_fx=total_fx,
            total_fy=total_fy,
            total_fz=total_fz,
            contact_active=self._contact_active,
            contact_score=contact_score,
            baseline_ready=True,
            active_mode="simulation",
            source_connected=True,
            source_name="simulator",
            frame=None,
            parser_state="simulation_active",
            mapping_state="simulation_3x3",
            status_text="simulation active",
            raw_frame_hex="",
        )
        self._phase += 0.08

    def _publish_empty(
        self,
        *,
        active_mode: str,
        source_connected: bool,
        parser_state: str,
        mapping_state: str,
        status_text: str,
        frame: Optional[ParsedFrame] = None,
    ) -> None:
        self._publish_message(
            rows=0,
            cols=0,
            forces=[],
            forces_fx=[],
            forces_fy=[],
            forces_fz=[],
            total_fx=0.0,
            total_fy=0.0,
            total_fz=0.0,
            contact_active=False,
            contact_score=0.0,
            baseline_ready=self._baseline_ready(),
            active_mode=active_mode,
            source_connected=source_connected,
            source_name=self.serial_port or "stm32-cdc",
            frame=frame,
            parser_state=parser_state,
            mapping_state=mapping_state,
            status_text=status_text,
            raw_frame_hex=str(frame.raw_frame_hex if frame is not None else ""),
        )

    def _publish_message(
        self,
        *,
        rows: int,
        cols: int,
        forces: List[float],
        forces_fx: List[float],
        forces_fy: List[float],
        forces_fz: List[float],
        total_fx: float,
        total_fy: float,
        total_fz: float,
        contact_active: bool,
        contact_score: float,
        baseline_ready: bool,
        active_mode: str,
        source_connected: bool,
        source_name: str,
        frame: Optional[ParsedFrame],
        parser_state: str,
        mapping_state: str,
        status_text: str,
        raw_frame_hex: str,
    ) -> None:
        msg = TactileRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tactile_sensor"
        msg.rows = max(0, rows)
        msg.cols = max(0, cols)
        msg.sequence_id = self._sequence_id
        msg.frame_id = "m2020_taxel_grid" if forces else "tactile_diagnostics"
        msg.forces = [float(item) for item in forces]
        msg.forces_fx = [float(item) for item in forces_fx]
        msg.forces_fy = [float(item) for item in forces_fy]
        msg.forces_fz = [float(item) for item in forces_fz]
        msg.torques_tx = [0.0] * len(forces)
        msg.torques_ty = [0.0] * len(forces)
        msg.torques_tz = [0.0] * len(forces)
        msg.total_fx = float(total_fx)
        msg.total_fy = float(total_fy)
        msg.total_fz = float(total_fz)
        msg.contact_active = bool(contact_active)
        msg.contact_score = float(contact_score)
        msg.baseline_ready = bool(baseline_ready)
        msg.sensor_layout = self.sensor_layout
        msg.requested_mode = self.requested_mode
        msg.active_mode = active_mode
        msg.source_name = source_name
        msg.source_connected = bool(source_connected)
        msg.device_addr = int(frame.device_addr if frame is not None else self.device_addr)
        msg.function_code = int(frame.function_code if frame is not None else 0)
        msg.start_addr = int(frame.start_addr if frame is not None else 0)
        msg.returned_data_len_hint = int(
            frame.returned_data_len_hint if frame is not None else 0
        )
        msg.frame_len_declared = int(frame.frame_len_declared if frame is not None else 0)
        msg.payload_len_observed = int(len(frame.payload) if frame is not None else 0)
        msg.payload_value_count = int(len(forces))
        msg.transport_rate_hz = float(self._transport_rate_hz)
        msg.publish_rate_hz = float(self.publish_rate_hz)
        msg.lrc_ok = bool(frame.lrc_ok if frame is not None else active_mode == "simulation")
        msg.parser_state = str(parser_state or "")
        msg.mapping_state = str(mapping_state or "")
        msg.status_text = str(status_text or "")
        msg.raw_frame_hex = str(raw_frame_hex or "")

        self.tactile_pub.publish(msg)
        self._sequence_id += 1
        self._last_parser_state = msg.parser_state
        self._last_mapping_state = msg.mapping_state
        self._last_status_text = msg.status_text
        self._last_output_updated_at = time.time()
        self._last_output_active_mode = active_mode
        self._last_output_source_connected = bool(source_connected)
        self._last_output_total = [float(total_fx), float(total_fy), float(total_fz)]
        self._last_output_fx = [float(item) for item in forces_fx]
        self._last_output_fy = [float(item) for item in forces_fy]
        self._last_output_fz = [float(item) for item in forces_fz]

    def _has_recent_output(self) -> bool:
        if self._last_output_updated_at <= 0.0:
            return False
        if (time.time() - self._last_output_updated_at) > max(2.0, 4.0 / self.publish_rate_hz):
            return False
        return self._last_output_source_connected

    def _clear_tare_state(self) -> None:
        self._tare_active = False
        self._tare_updated_at = 0.0
        self._tare_total_offset = [0.0, 0.0, 0.0]
        self._tare_fx_offset = []
        self._tare_fy_offset = []
        self._tare_fz_offset = []

    def _apply_tare_offsets(self, values: List[float], offsets: List[float]) -> List[float]:
        if not values or not self._tare_active:
            return list(values)
        if len(values) != len(offsets):
            return list(values)
        return [float(value) - float(offset) for value, offset in zip(values, offsets)]

    def _apply_tare_totals(
        self, total_fx: float, total_fy: float, total_fz: float
    ) -> Tuple[float, float, float]:
        if not self._tare_active or len(self._tare_total_offset) != 3:
            return total_fx, total_fy, total_fz
        return (
            float(total_fx) - float(self._tare_total_offset[0]),
            float(total_fy) - float(self._tare_total_offset[1]),
            float(total_fz) - float(self._tare_total_offset[2]),
        )

    def _publish_health(self) -> None:
        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "system"
        msg.node_name = self.get_name()
        msg.cpu_percent = 0.0
        msg.memory_percent = 0.0

        if self.requested_mode == "simulation":
            msg.healthy = True
            msg.level = 0
            msg.message = "tactile simulation active"
        else:
            recently_received = (
                self._last_good_frame_stamp > 0.0
                and (time.time() - self._last_good_frame_stamp)
                <= max(1.0, 2.5 / self.publish_rate_hz)
            )
            if recently_received:
                msg.healthy = True
                msg.level = 0
                msg.message = self._last_status_text or "hardware stream active"
            elif self._serial is not None and getattr(self._serial, "is_open", False):
                msg.healthy = False
                msg.level = 1
                msg.message = (
                    self._last_status_text or "serial connected but no recent tactile frame"
                )
            else:
                msg.healthy = False
                msg.level = 2
                msg.message = (
                    self._connect_error or "hardware mode requested but serial is disconnected"
                )

        self.health_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._disconnect_serial()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
