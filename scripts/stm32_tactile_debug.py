import argparse
import json
import os
import re
import time
from pathlib import Path
from typing import Any

try:
    import serial
except ModuleNotFoundError:
    serial = None


DEFAULT_PORT = os.environ.get("STM32_PORT") or ("COM4" if os.name == "nt" else "/dev/ttyS4")
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 1.0

TRAW_RE = re.compile(r"^(TRAWX|TRAW) n=(\d+)\s+(.+)$")
TREAD_OK_RE = re.compile(r"^(TREADX|TREAD) OK len=(\d+)\s+(.+)$")


def bytes_to_hex(data: list[int]) -> str:
    return " ".join(f"{value:02X}" for value in data)


def to_s8(value: int) -> int:
    return value - 256 if value >= 128 else value


def calc_lrc(data: list[int]) -> int:
    return (-sum(data)) & 0xFF


def parse_hex_bytes(hex_blob: str) -> list[int]:
    tokens = hex_blob.strip().split()
    return [int(token, 16) for token in tokens]


def parse_frame_bytes(raw_bytes: list[int], declared_total_len: int, source: str) -> dict[str, Any]:
    frame: dict[str, Any] = {
        "source": source,
        "declared_total_len": declared_total_len,
        "actual_total_len": len(raw_bytes),
        "raw_bytes": raw_bytes,
        "raw_hex": bytes_to_hex(raw_bytes),
        "declared_total_match": len(raw_bytes) == declared_total_len,
        "header_ok": len(raw_bytes) >= 2 and raw_bytes[0] == 0xAA and raw_bytes[1] == 0x55,
        "frame_len": None,
        "expected_total_len": None,
        "expected_total_match": None,
        "device_addr": None,
        "reserved": None,
        "function": None,
        "start_addr": None,
        "declared_data_len": None,
        "status": None,
        "payload": [],
        "payload_hex": "",
        "payload_len": 0,
        "payload_27": [],
        "payload_27_hex": "",
        "tail_5": [],
        "tail_5_hex": "",
        "lrc_recv": None,
        "lrc_calc": None,
        "lrc_ok": None,
        "valid_frame": False,
    }

    if len(raw_bytes) >= 4:
        frame_len = raw_bytes[2] | (raw_bytes[3] << 8)
        frame["frame_len"] = frame_len
        frame["expected_total_len"] = 4 + frame_len + 1
        frame["expected_total_match"] = frame["expected_total_len"] == len(raw_bytes)

    if len(raw_bytes) >= 14:
        frame["device_addr"] = raw_bytes[4]
        frame["reserved"] = raw_bytes[5]
        frame["function"] = raw_bytes[6]
        frame["start_addr"] = (
            raw_bytes[7]
            | (raw_bytes[8] << 8)
            | (raw_bytes[9] << 16)
            | (raw_bytes[10] << 24)
        )
        frame["declared_data_len"] = raw_bytes[11] | (raw_bytes[12] << 8)
        frame["status"] = raw_bytes[13]

    if len(raw_bytes) >= 15:
        payload = raw_bytes[14:-1]
        frame["payload"] = payload
        frame["payload_hex"] = bytes_to_hex(payload)
        frame["payload_len"] = len(payload)
        frame["payload_27"] = payload[:27]
        frame["payload_27_hex"] = bytes_to_hex(frame["payload_27"])
        frame["tail_5"] = payload[27:32]
        frame["tail_5_hex"] = bytes_to_hex(frame["tail_5"])

    if raw_bytes:
        frame["lrc_recv"] = raw_bytes[-1]
        frame["lrc_calc"] = calc_lrc(raw_bytes[:-1])
        frame["lrc_ok"] = frame["lrc_calc"] == frame["lrc_recv"]

    frame["valid_frame"] = bool(
        frame["header_ok"]
        and frame["expected_total_match"]
        and frame["lrc_ok"]
        and frame["payload_len"] >= 0
    )
    return frame


def parse_traw_response(resp: str) -> dict[str, Any]:
    if resp.startswith("TRAWX FAIL"):
        return {"source": "TRAWX", "ok": False, "error": resp, "kind": "traw_fail"}
    if resp.startswith("TRAW FAIL"):
        return {"source": "TRAW", "ok": False, "error": resp, "kind": "traw_fail"}

    match = TRAW_RE.match(resp)
    if not match:
        return {"source": "TRAW", "ok": False, "error": resp, "kind": "traw_unparsed"}

    source = match.group(1)
    declared_total_len = int(match.group(2))
    try:
        raw_bytes = parse_hex_bytes(match.group(3))
    except ValueError:
        return {"source": source, "ok": False, "error": resp, "kind": "traw_bad_hex"}

    frame = parse_frame_bytes(raw_bytes, declared_total_len, source)
    frame["ok"] = True
    return frame


def parse_tread_response(resp: str) -> dict[str, Any]:
    if resp.startswith("TREADX FAIL"):
        return {"source": "TREADX", "ok": False, "error": resp, "kind": "tread_fail"}
    if resp.startswith("TREAD FAIL"):
        return {"source": "TREAD", "ok": False, "error": resp, "kind": "tread_fail"}

    match = TREAD_OK_RE.match(resp)
    if not match:
        return {"source": "TREAD", "ok": False, "error": resp, "kind": "tread_unparsed"}

    source = match.group(1)
    declared_total_len = int(match.group(2))
    try:
        raw_bytes = parse_hex_bytes(match.group(3))
    except ValueError:
        return {"source": source, "ok": False, "error": resp, "kind": "tread_bad_hex"}

    frame = parse_frame_bytes(raw_bytes, declared_total_len, source)
    frame["ok"] = True
    return frame


def parse_tstat_response(resp: str) -> dict[str, Any]:
    stats: dict[str, Any] = {"source": "TSTAT", "raw": resp}
    if not resp.startswith("TSTAT "):
        return stats
    for token in resp.split()[1:]:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            stats[key] = int(value)
        except ValueError:
            stats[key] = value
    return stats


def segment_activity(frames: list[dict[str, Any]], key: str) -> list[dict[str, Any]]:
    if not frames:
        return []

    max_len = max(len(frame.get(key, [])) for frame in frames)
    activity: list[dict[str, Any]] = []
    for idx in range(max_len):
        values = [frame[key][idx] for frame in frames if len(frame.get(key, [])) > idx]
        unique = sorted(set(values))
        if len(unique) > 1 or any(value != 0 for value in unique):
            activity.append(
                {
                    "index": idx,
                    "values_hex": [f"{value:02X}" for value in unique],
                    "values_s8": [to_s8(value) for value in unique],
                }
            )
    return activity


def build_candidate_triplets(payload_27: list[int]) -> list[dict[str, int]]:
    triplets: list[dict[str, int]] = []
    for idx in range(0, min(len(payload_27), 27), 3):
        if idx + 2 >= len(payload_27):
            break
        triplets.append(
            {
                "slot": idx // 3,
                "fx_s8": to_s8(payload_27[idx]),
                "fy_s8": to_s8(payload_27[idx + 1]),
                "fz_u8": payload_27[idx + 2],
            }
        )
    return triplets


def build_tactile_commands(dev: int, addr: int | None, length: int | None) -> tuple[str, str]:
    if addr is None and length is None:
        return f"TRAW {dev}", f"TREAD {dev}"

    if addr is None or length is None:
        raise ValueError("--addr and --length must be provided together")

    return f"TRAWX {dev} {addr} {length}", f"TREADX {dev} {addr} {length}"


def report_frame(frame: dict[str, Any], baseline_payload: list[int] | None, show_json: bool, show_triplets: bool) -> None:
    if not frame.get("ok", False):
        print(f"   parse: {frame.get('kind')} {frame.get('error')}")
        return

    print(
        "   frame:"
        f" total={frame['actual_total_len']}"
        f" declared={frame['declared_total_len']}"
        f" expected={frame['expected_total_len']}"
        f" header={'ok' if frame['header_ok'] else 'bad'}"
        f" lrc={'ok' if frame['lrc_ok'] else 'bad'}"
    )
    print(
        "   fields:"
        f" addr={frame['device_addr']}"
        f" func=0x{frame['function']:02X}" if frame["function"] is not None else "   fields: unavailable",
        end="",
    )
    if frame["function"] is not None:
        print(
            f" start={frame['start_addr']}"
            f" declared_data_len={frame['declared_data_len']}"
            f" status={frame['status']}"
        )
    else:
        print()

    if frame["payload"]:
        print(f"   payload[{frame['payload_len']}]: {frame['payload_hex'] or '(empty)'}")
        if frame["payload_27"]:
            print(f"   tactile27[{len(frame['payload_27'])}]: {frame['payload_27_hex'] or '(empty)'}")
        if frame["tail_5"]:
            print(f"   tail_after_27[{len(frame['tail_5'])}]: {frame['tail_5_hex'] or '(empty)'}")

    if baseline_payload is not None and frame["payload"]:
        changed = [
            idx
            for idx, (old_value, new_value) in enumerate(zip(baseline_payload, frame["payload"]))
            if old_value != new_value
        ]
        if changed:
            print(f"   payload_delta_vs_baseline: {changed}")
        else:
            print("   payload_delta_vs_baseline: none")

    if show_triplets and len(frame["payload_27"]) >= 27:
        print("   candidate_triplets_9x3:")
        for triplet in build_candidate_triplets(frame["payload_27"]):
            print(
                "    "
                f"slot={triplet['slot']} "
                f"fx={triplet['fx_s8']} "
                f"fy={triplet['fy_s8']} "
                f"fz={triplet['fz_u8']}"
            )

    if show_json:
        print("   json:")
        print(json.dumps(frame, ensure_ascii=False, indent=2))


def report_summary(frames: list[dict[str, Any]]) -> None:
    parsed_frames = [frame for frame in frames if frame.get("ok", False)]
    valid_frames = [frame for frame in parsed_frames if frame.get("valid_frame")]

    print(f"== {parsed_frames[0]['source'] if parsed_frames else 'TRAW'} Summary ==")
    print(f"parsed_frames={len(parsed_frames)} valid_frames={len(valid_frames)} total_frames={len(frames)}")
    if not parsed_frames:
        return

    print(
        "invalid_breakdown:"
        f" bad_header={sum(1 for frame in parsed_frames if frame.get('ok') and not frame.get('header_ok'))}"
        f" bad_lrc={sum(1 for frame in parsed_frames if frame.get('ok') and frame.get('header_ok') and not frame.get('lrc_ok'))}"
        f" len_mismatch={sum(1 for frame in parsed_frames if frame.get('ok') and not frame.get('expected_total_match'))}"
    )

    if not valid_frames:
        return

    payload_activity = segment_activity(valid_frames, "payload")
    tactile27_activity = segment_activity(valid_frames, "payload_27")
    tail5_activity = segment_activity(valid_frames, "tail_5")

    print(f"payload_active_bytes={len(payload_activity)}")
    for item in payload_activity:
        print(
            f"  payload[{item['index']}]:"
            f" hex={','.join(item['values_hex'])}"
            f" s8={','.join(str(value) for value in item['values_s8'])}"
        )

    print(f"tactile27_active_bytes={len(tactile27_activity)}")
    for item in tactile27_activity:
        print(
            f"  tactile27[{item['index']}]:"
            f" hex={','.join(item['values_hex'])}"
            f" s8={','.join(str(value) for value in item['values_s8'])}"
        )

    print(f"tail_after_27_active_bytes={len(tail5_activity)}")
    for item in tail5_activity:
        print(
            f"  tail_after_27[{item['index']}]:"
            f" hex={','.join(item['values_hex'])}"
            f" s8={','.join(str(value) for value in item['values_s8'])}"
        )


def send_cmd(ser: Any, cmd: str, wait_s: float = 0.1) -> str:
    if not cmd.endswith("\n"):
        cmd += "\n"

    print(f">> {cmd.strip()}")
    ser.write(cmd.encode("utf-8"))
    ser.flush()
    time.sleep(wait_s)

    resp = ser.readline().decode(errors="ignore").strip()
    print(f"<< {resp}")
    return resp


def maybe_append_log(path: Path | None, data: dict[str, Any]) -> None:
    if path is None:
        return
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(data, ensure_ascii=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="STM32 tactile UART/CDC debug helper")
    parser.add_argument("--port", default=DEFAULT_PORT, help="CDC serial port, e.g. COM4")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="CDC baud rate")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Serial timeout in seconds")
    parser.add_argument("--dev", type=int, default=1, help="Tactile device address")
    parser.add_argument("--addr", type=lambda value: int(value, 0), help="Optional read start address, e.g. 1008 or 0x3F0")
    parser.add_argument("--length", type=lambda value: int(value, 0), help="Optional read length; use with --addr")
    parser.add_argument("--loops", type=int, default=30, help="How many TRAW commands to send")
    parser.add_argument("--raw-wait", type=float, default=0.2, help="Wait time after each TRAW")
    parser.add_argument("--skip-tread", action="store_true", help="Skip the final TREAD command")
    parser.add_argument("--show-json", action="store_true", help="Print parsed frame JSON")
    parser.add_argument("--show-triplets", action="store_true", help="Show candidate 9x3 grouping for the first 27 payload bytes")
    parser.add_argument("--no-summary", action="store_true", help="Skip final TRAW summary")
    parser.add_argument("--jsonl", help="Optional JSONL log file for parsed responses")
    args = parser.parse_args()

    log_path = Path(args.jsonl) if args.jsonl else None

    try:
        raw_cmd, frame_cmd = build_tactile_commands(args.dev, args.addr, args.length)
    except ValueError as exc:
        print(exc)
        return

    if serial is None:
        print("pyserial is not installed. Install it with: python -m pip install pyserial")
        return

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as exc:
        print("Serial open failed:", exc)
        return

    print("Serial opened:", ser.name)
    time.sleep(1.0)

    traw_frames: list[dict[str, Any]] = []
    baseline_payload: list[int] | None = None

    try:
        maybe_append_log(log_path, {"event": "open", "port": ser.name, "baud": args.baud})

        send_cmd(ser, "PING")
        send_cmd(ser, "VER")
        send_cmd(ser, "TSTAT RESET")

        for idx in range(args.loops):
            print(f"-- {raw_cmd} loop {idx + 1}/{args.loops}")
            resp = send_cmd(ser, raw_cmd, wait_s=args.raw_wait)
            parsed = parse_traw_response(resp)
            traw_frames.append(parsed)
            maybe_append_log(log_path, {"cmd": raw_cmd, "raw": resp, "parsed": parsed})

            if parsed.get("ok") and parsed.get("valid_frame") and baseline_payload is None:
                baseline_payload = list(parsed["payload"])

            report_frame(parsed, baseline_payload, args.show_json, args.show_triplets)

        tstat_resp = send_cmd(ser, "TSTAT")
        tstat = parse_tstat_response(tstat_resp)
        maybe_append_log(log_path, {"cmd": "TSTAT", "raw": tstat_resp, "parsed": tstat})

        if not args.skip_tread:
            tread_resp = send_cmd(ser, frame_cmd, wait_s=0.2)
            tread = parse_tread_response(tread_resp)
            maybe_append_log(log_path, {"cmd": frame_cmd, "raw": tread_resp, "parsed": tread})
            report_frame(tread, baseline_payload, args.show_json, args.show_triplets)

        if not args.no_summary:
            report_summary(traw_frames)

    finally:
        ser.close()
        maybe_append_log(log_path, {"event": "close"})
        print("Serial closed")


if __name__ == "__main__":
    main()
