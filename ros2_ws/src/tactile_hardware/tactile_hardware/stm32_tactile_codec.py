from __future__ import annotations

import re
from dataclasses import dataclass
from typing import List, Sequence


TREAD_OK_PATTERN = re.compile(r"^(TREADX|TREAD)\s+OK\s+len=(\d+)\s+([0-9A-Fa-f\s]+)$")
TRAW_PATTERN = re.compile(r"^(TRAWX|TRAW)\s+n=(\d+)\s+([0-9A-Fa-f\s]+)$")


def to_s8(value: int) -> int:
    raw = int(value) & 0xFF
    return raw - 256 if raw >= 128 else raw


def calc_lrc(data: bytes) -> int:
    return (-sum(int(item) for item in data)) & 0xFF


@dataclass
class ParsedFrame:
    source: str
    raw: bytes
    raw_frame_hex: str
    observed_len: int
    expected_total_len: int
    frame_len_declared: int
    device_addr: int
    function_code: int
    start_addr: int
    returned_data_len_hint: int
    status: int
    payload: bytes
    lrc_ok: bool
    parser_state: str
    status_text: str


@dataclass
class TotalsData:
    fx: int
    fy: int
    fz: int


@dataclass
class TaxelData:
    fx: int
    fy: int
    fz: int


def parse_reply_line(line: str) -> ParsedFrame:
    text = str(line or "").strip()
    if not text:
        return _error_frame("empty_reply", "reply is empty")

    if " FAIL" in text:
        source = text.split()[0]
        return _error_frame("device_fail", text, source=source)

    tread_match = TREAD_OK_PATTERN.match(text)
    if tread_match:
        return _parse_frame_match(
            source=tread_match.group(1),
            observed_len=int(tread_match.group(2)),
            raw_frame_hex=" ".join(tread_match.group(3).split()).upper(),
        )

    traw_match = TRAW_PATTERN.match(text)
    if traw_match:
        return _parse_frame_match(
            source=traw_match.group(1),
            observed_len=int(traw_match.group(2)),
            raw_frame_hex=" ".join(traw_match.group(3).split()).upper(),
        )

    return _error_frame("invalid_reply_prefix", text)


def decode_totals(payload: Sequence[int] | bytes) -> TotalsData:
    values = bytes(payload)
    if len(values) < 3:
        raise ValueError(f"totals payload too short: {len(values)}")
    return TotalsData(
        fx=to_s8(values[0]),
        fy=to_s8(values[1]),
        fz=int(values[2]),
    )


def decode_taxels(payload: Sequence[int] | bytes) -> List[TaxelData]:
    values = bytes(payload)
    if len(values) < 27:
        raise ValueError(f"taxel payload too short: {len(values)}")
    if len(values) != 27:
        values = values[:27]
    decoded: List[TaxelData] = []
    for index in range(0, 27, 3):
        decoded.append(
            TaxelData(
                fx=to_s8(values[index]),
                fy=to_s8(values[index + 1]),
                fz=int(values[index + 2]),
            )
        )
    return decoded


def _parse_frame_match(*, source: str, observed_len: int, raw_frame_hex: str) -> ParsedFrame:
    try:
        raw = bytes.fromhex(raw_frame_hex)
    except ValueError as exc:
        return _error_frame(
            "hex_decode_error",
            str(exc),
            source=source,
            raw_frame_hex=raw_frame_hex,
            observed_len=observed_len,
        )

    if len(raw) < 15:
        return _error_frame(
            "frame_too_short",
            f"observed={len(raw)} bytes",
            source=source,
            raw=raw,
            raw_frame_hex=raw_frame_hex,
            observed_len=observed_len,
        )

    if raw[0:2] != b"\xAA\x55":
        return _error_frame(
            "invalid_header",
            f"header={raw[0:2].hex().upper()}",
            source=source,
            raw=raw,
            raw_frame_hex=raw_frame_hex,
            observed_len=observed_len,
        )

    frame_len_declared = int.from_bytes(raw[2:4], byteorder="little", signed=False)
    expected_total_len = frame_len_declared + 5
    device_addr = int(raw[4])
    function_code = int(raw[6])
    start_addr = int.from_bytes(raw[7:11], byteorder="little", signed=False)
    returned_data_len_hint = int.from_bytes(raw[11:13], byteorder="little", signed=False)
    status = int(raw[13])
    payload = raw[14:-1]
    observed_lrc = int(raw[-1])
    computed_lrc = calc_lrc(raw[:-1])
    lrc_ok = observed_lrc == computed_lrc

    problems: List[str] = []
    if len(raw) != observed_len:
        problems.append(f"reply_count={observed_len}, decoded={len(raw)}")
    if len(raw) != expected_total_len:
        problems.append(f"declared_total={expected_total_len}, observed={len(raw)}")
    if not lrc_ok:
        problems.append(f"LRC expected={computed_lrc:02X} observed={observed_lrc:02X}")
    if status != 0:
        problems.append(f"device_status={status}")

    parser_state = "frame_ok" if not problems else "frame_warn"
    return ParsedFrame(
        source=source,
        raw=raw,
        raw_frame_hex=raw_frame_hex,
        observed_len=observed_len,
        expected_total_len=expected_total_len,
        frame_len_declared=frame_len_declared,
        device_addr=device_addr,
        function_code=function_code,
        start_addr=start_addr,
        returned_data_len_hint=returned_data_len_hint,
        status=status,
        payload=payload,
        lrc_ok=lrc_ok,
        parser_state=parser_state,
        status_text="; ".join(problems) if problems else "frame parsed",
    )


def _error_frame(
    parser_state: str,
    status_text: str,
    *,
    source: str = "",
    raw: bytes = b"",
    raw_frame_hex: str = "",
    observed_len: int = 0,
) -> ParsedFrame:
    return ParsedFrame(
        source=source,
        raw=raw,
        raw_frame_hex=raw_frame_hex,
        observed_len=observed_len,
        expected_total_len=0,
        frame_len_declared=0,
        device_addr=0,
        function_code=0,
        start_addr=0,
        returned_data_len_hint=0,
        status=0,
        payload=b"",
        lrc_ok=False,
        parser_state=parser_state,
        status_text=str(status_text or ""),
    )
