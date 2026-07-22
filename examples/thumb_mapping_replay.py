#!/usr/bin/env python3
"""Replay recorded Unity packets through legacy and calibrated thumb mappings."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

from thumb_calibration import euler_to_quaternion, quaternion_log_deg, multiply


JOINT_INDICES = (1, 2, 3, 5, 9, 13, 17)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def linear(value: float, low: float, high: float) -> float:
    return clamp((value - low) / (high - low), 0.0, 1.0)


def sensitivity(target: float, gain: float) -> int:
    return int(round(clamp(1000.0 + (target - 1000.0) * gain, 0.0, 2000.0)))


def legacy_thumb(rot: Sequence[float]) -> Tuple[float, float, int, int]:
    mcp, ip = rot[6:9], rot[9:12]
    yaw_feature = wrap(float(ip[2]) - float(mcp[2]))
    yaw_base = linear(yaw_feature, -12.0, 5.0) * 2000.0
    yaw_target = sensitivity(yaw_base, 1.35)
    pitch_feature = abs(wrap(float(ip[1]) - float(mcp[1])))
    pitch_base = 2000.0 * (1.0 - linear(pitch_feature, 40.0, 80.0))
    yaw_ratio = linear(yaw_feature, -12.0, 5.0)
    pitch_gain = 0.5 + 1.5 * yaw_ratio
    pitch_target = sensitivity(pitch_base, pitch_gain)
    return yaw_feature, pitch_feature, yaw_target, pitch_target


def fixed_finger_target(feature: float, low: float, high: float) -> int:
    ratio = linear(feature, low, high)
    if ratio <= 0.06:
        t = 0.0
    elif ratio >= 0.88:
        t = 1.0
    else:
        middle = (ratio - 0.06) / 0.82
        t = middle * middle * (3.0 - 2.0 * middle)
    return int(round(2000.0 * (1.0 - t)))


def legacy_fingers(rot: Sequence[float], previous: List[int]) -> List[int]:
    ranges = ((5, 6, 9.75, 58.67), (9, 10, 5.11, 57.85), (13, 14, 6.50, 58.21), (17, 18, 13.0, 59.29))
    result = []
    for output_index, (mcp, pip, low, high) in enumerate(ranges):
        m = math.sqrt(sum(float(rot[mcp * 3 + axis]) ** 2 for axis in range(3)))
        p = math.sqrt(sum(float(rot[pip * 3 + axis]) ** 2 for axis in range(3)))
        target = fixed_finger_target(0.35 * m + 0.65 * p, low, high)
        target = max(previous[output_index] - 100, min(previous[output_index] + 100, target))
        result.append(target)
    return result


def load_calibration(path: Path) -> dict:
    config = json.loads(path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("unsupported calibration schema")
    return config


def calibrated_thumb(rot: Sequence[float], config: dict) -> Tuple[float, float, float, float, int, int]:
    order = config["euler_order"]
    references = np.asarray(config["reference_quaternions_wxyz"], dtype=float)
    row = []
    for selected, joint in enumerate(config["feature_joint_indices"]):
        current = euler_to_quaternion(rot[joint * 3 : joint * 3 + 3], order)
        reference = references[selected]
        delta = multiply(np.array([reference[0], -reference[1], -reference[2], -reference[3]]), current)
        row.extend(quaternion_log_deg(delta))
    feature = np.asarray(row, dtype=float)
    basis = np.asarray(config["nuisance_basis"], dtype=float)
    residual = feature if basis.size == 0 else feature - (feature @ basis.T) @ basis
    projected = np.array([residual @ np.asarray(config["yaw_axis"]), residual @ np.asarray(config["pitch_axis"])])
    scores = np.asarray(config["demix_matrix"], dtype=float) @ projected

    def map_one(score: float, channel: str) -> Tuple[float, int]:
        limits = config["score_ranges"][channel]
        ratio = linear(score, float(limits["open"]), float(limits["close"]))
        open_deadzone = float(config["mapping"]["open_deadzone"])
        close_deadzone = float(config["mapping"]["close_deadzone"])
        if ratio <= open_deadzone:
            mapped = 0.0
        elif ratio >= 1.0 - close_deadzone:
            mapped = 1.0
        else:
            middle = (ratio - open_deadzone) / (1.0 - open_deadzone - close_deadzone)
            mapped = middle * middle * (3.0 - 2.0 * middle)
        return mapped, int(round(2000.0 * (1.0 - mapped)))

    yaw_ratio, yaw_target = map_one(float(scores[0]), "yaw")
    pitch_ratio, pitch_target = map_one(float(scores[1]), "pitch")
    return float(scores[0]), float(scores[1]), yaw_ratio, pitch_ratio, yaw_target, pitch_target


def load_actual(path: Path | None) -> Tuple[List[int], Dict[int, Dict[int, dict]]]:
    if path is None or not path.exists():
        return [], {}
    grouped: Dict[int, Dict[int, dict]] = {}
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            wall = int(float(row["WALL_TIME_MS"]))
            actuator_id = int(str(row["ID"]).upper().removeprefix("ID"))
            grouped.setdefault(wall, {})[actuator_id] = row
    return sorted(grouped), grouped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("unity_jsonl", type=Path)
    parser.add_argument("calibration", type=Path)
    parser.add_argument("--actual-csv", type=Path)
    parser.add_argument("--output", type=Path, default=Path("thumb_mapping_replay.csv"))
    parser.add_argument("--plot", type=Path, default=Path("thumb_mapping_replay.png"))
    parser.add_argument("--max-frames", type=int)
    args = parser.parse_args()

    config = load_calibration(args.calibration)
    actual_times, actual = load_actual(args.actual_csv)
    rows = []
    previous_fingers: List[int] | None = None
    nonthumb_mismatches = 0
    with args.unity_jsonl.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            payload = record.get("unity_payload", {})
            rot = payload.get("rightRotations")
            baseline = record.get("mapped_target_units")
            if not (isinstance(rot, list) and len(rot) >= 63 and isinstance(baseline, list) and len(baseline) == 6):
                continue
            yaw_feature, pitch_feature, legacy_yaw, legacy_pitch = legacy_thumb(rot)
            yaw_score, pitch_score, yaw_ratio, pitch_ratio, new_yaw, new_pitch = calibrated_thumb(rot, config)
            if previous_fingers is None:
                previous_fingers = [int(value) for value in baseline[2:]]
                replayed_fingers = list(previous_fingers)
            else:
                replayed_fingers = legacy_fingers(rot, previous_fingers)
                if replayed_fingers != [int(value) for value in baseline[2:]]:
                    nonthumb_mismatches += 1
                previous_fingers = replayed_fingers

            actual_row = {}
            recv_ms = int(record["recv_time_ns"]) // 1_000_000
            if actual_times:
                insertion = bisect.bisect_left(actual_times, recv_ms)
                candidates = actual_times[max(0, insertion - 1) : min(len(actual_times), insertion + 1)]
                nearest = min(candidates, key=lambda value: abs(value - recv_ms)) if candidates else None
                if nearest is not None and abs(nearest - recv_ms) <= 100:
                    actual_row = actual[nearest]
            rows.append(
                {
                    "recv_time_ns": record["recv_time_ns"],
                    "frame_id": payload.get("frameId", ""),
                    "legacy_yaw_feature": yaw_feature,
                    "legacy_pitch_feature": pitch_feature,
                    "legacy_yaw_target": legacy_yaw,
                    "legacy_pitch_target": legacy_pitch,
                    "new_yaw_score": yaw_score,
                    "new_pitch_score": pitch_score,
                    "new_yaw_ratio": yaw_ratio,
                    "new_pitch_ratio": pitch_ratio,
                    "new_yaw_target": new_yaw,
                    "new_pitch_target": new_pitch,
                    "baseline_index": baseline[2],
                    "baseline_middle": baseline[3],
                    "baseline_ring": baseline[4],
                    "baseline_pinky": baseline[5],
                    "replay_index": replayed_fingers[0],
                    "replay_middle": replayed_fingers[1],
                    "replay_ring": replayed_fingers[2],
                    "replay_pinky": replayed_fingers[3],
                    "actual_yaw": actual_row.get(1, {}).get("REAL", ""),
                    "actual_pitch": actual_row.get(2, {}).get("REAL", ""),
                    "command_yaw": actual_row.get(1, {}).get("CMD", ""),
                    "command_pitch": actual_row.get(2, {}).get("CMD", ""),
                }
            )
            if args.max_frames and len(rows) >= args.max_frames:
                break
    if not rows:
        raise RuntimeError("no accepted rightRotations frames found")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    try:
        import matplotlib.pyplot as plt

        t0 = int(rows[0]["recv_time_ns"])
        timeline = [(int(row["recv_time_ns"]) - t0) / 1e9 for row in rows]
        figure, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        for axis, channel in zip(axes, ("yaw", "pitch")):
            axis.plot(timeline, [row[f"legacy_{channel}_target"] for row in rows], label="legacy target", alpha=0.7)
            axis.plot(timeline, [row[f"new_{channel}_target"] for row in rows], label="calibrated target", linewidth=1.2)
            actual_values = [float(row[f"actual_{channel}"]) if row[f"actual_{channel}"] != "" else np.nan for row in rows]
            axis.plot(timeline, actual_values, label="WA100 actual", alpha=0.7)
            axis.set_ylabel(f"thumb {channel} units")
            axis.set_ylim(-50, 2050)
            axis.grid(True, alpha=0.25)
            axis.legend(loc="upper right")
        axes[-1].set_xlabel("time (s)")
        figure.tight_layout()
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.plot, dpi=150)
        plt.close(figure)
    except ImportError:
        print("matplotlib is unavailable; CSV was generated without a plot")

    print(f"frames={len(rows)} nonthumb_replay_mismatches={nonthumb_mismatches}")
    print(f"CSV: {args.output.resolve()}")
    if args.plot.exists():
        print(f"Plot: {args.plot.resolve()}")
    return 0 if nonthumb_mismatches == 0 else 3


if __name__ == "__main__":
    raise SystemExit(main())
