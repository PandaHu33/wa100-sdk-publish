#!/usr/bin/env python3
"""Add four-finger open/close endpoints to the unified Hi5 calibration JSON."""

from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime
from pathlib import Path

import numpy as np


CHANNELS = {
    "index": (5, 6),
    "middle": (9, 10),
    "ring": (13, 14),
    "pinky": (17, 18),
}


def features(rotations: list[float], mcp_weight: float = 0.35) -> dict[str, float]:
    values = np.asarray(rotations, dtype=float).reshape(-1, 3)
    if values.shape[0] < 21 or not np.all(np.isfinite(values[:21])):
        raise ValueError("rightRotations must contain 21 finite Euler triplets")
    pip_weight = 1.0 - mcp_weight
    return {
        name: float(mcp_weight * np.linalg.norm(values[mcp]) + pip_weight * np.linalg.norm(values[pip]))
        for name, (mcp, pip) in CHANNELS.items()
    }


def collect(sock: socket.socket, duration_sec: float) -> list[dict[str, float]]:
    deadline, rows, last_frame = time.monotonic() + duration_sec, [], None
    while time.monotonic() < deadline:
        try:
            raw, _source = sock.recvfrom(65535)
            payload = json.loads(raw.decode("utf-8"))
            frame = int(payload.get("frameId", -1))
            if frame >= 0 and frame == last_frame:
                continue
            rows.append(features(payload["rightRotations"]))
            last_frame = frame
        except (socket.timeout, UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError):
            continue
    if len(rows) < max(20, int(duration_sec * 10)):
        raise RuntimeError(f"only {len(rows)} valid frames were collected")
    return rows


def endpoint(rows: list[dict[str, float]], name: str) -> float:
    return float(np.median([row[name] for row in rows]))


def main() -> int:
    default = Path(__file__).resolve().parents[1] / "build" / "bin" / "thumb_glove_calibration.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=default)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=25001)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()
    source = args.input.resolve()
    output = args.output.resolve() if args.output else source
    calibration = json.loads(source.read_text(encoding="utf-8"))
    if not bool((calibration.get("quality") or {}).get("quality_pass", False)):
        raise RuntimeError("thumb calibration quality_pass is false")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.5)
    try:
        input("Keep the wrist still, fully OPEN index-to-pinky, then press Enter...")
        opened = collect(sock, args.duration)
        input("Keep the wrist still, fully CLOSE index-to-pinky, then press Enter...")
        closed = collect(sock, args.duration)
    finally:
        sock.close()

    channels, quality = {}, {}
    for name in CHANNELS:
        open_value, close_value = endpoint(opened, name), endpoint(closed, name)
        span = close_value - open_value
        channels[name] = {
            "open_feature_deg": open_value,
            "close_feature_deg": close_value,
        }
        quality[name + "_span_deg"] = span
    quality_pass = all(value >= 5.0 for value in quality.values())
    mapping = {
        "method": "weighted_mcp_pip_rotation_magnitude",
        "mcp_weight": 0.35,
        "pip_weight": 0.65,
        "open_deadzone": 0.06,
        "close_deadzone": 0.12,
        "channels": channels,
        "quality": {**quality, "quality_pass": quality_pass},
    }
    if not quality_pass:
        failed = output.with_name(output.stem + ".finger-endpoints.failed.json")
        failed.write_text(json.dumps(mapping, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"Endpoint calibration failed; diagnostic: {failed}")
        return 2

    previous_id = str(calibration.get("calibration_id") or "")
    calibration["previous_calibration_id"] = previous_id
    calibration["calibration_id"] = "hi5-hand-" + datetime.now().strftime("%Y%m%d-%H%M%S")
    calibration["finger_mapping"] = mapping
    calibration["finger_mapping_updated_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(json.dumps(calibration, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(output)
    print(f"Unified glove calibration written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
