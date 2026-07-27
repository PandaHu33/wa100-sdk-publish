#!/usr/bin/env python3
"""Validate Hi5 bone semantics and wrist/finger decoupling from a JSONL replay."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def wrapped_delta(values: np.ndarray) -> np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def analyze(path: Path) -> dict:
    rotations, targets = [], []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                record = json.loads(line)
                payload = record.get("unity_payload") or {}
                values = payload.get("rightRotations")
                mapped = record.get("mapped_target_units")
                if (
                    record.get("accepted")
                    and isinstance(values, list)
                    and len(values) >= 63
                    and isinstance(mapped, list)
                    and len(mapped) == 6
                ):
                    rotations.append(values[:63])
                    targets.append(mapped)
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
    if len(rotations) < 2:
        raise ValueError("replay has fewer than two accepted rightRotations frames")
    rotation_array = np.asarray(rotations, dtype=float).reshape(-1, 21, 3)
    target_array = np.asarray(targets, dtype=float)
    delta = wrapped_delta(np.diff(rotation_array, axis=0))
    hand_step = np.linalg.norm(delta[:, 1], axis=1)
    local_finger_step = np.linalg.norm(delta[:, 2:], axis=(1, 2))
    target_step = np.max(np.abs(np.diff(target_array, axis=0)), axis=1)
    pure_wrist = (hand_step > 2.0) & (local_finger_step < 0.5)
    pure_wrist_target = target_step[pure_wrist]
    forearm_range = np.ptp(rotation_array[:, 0], axis=0)
    hand_range = np.ptp(rotation_array[:, 1], axis=0)
    return {
        "schema_version": 1,
        "source": str(path.resolve()),
        "accepted_frames": int(rotation_array.shape[0]),
        "bone_semantics": {"0": "ForeArm", "1": "Hand", "2-20": "local_finger_bones"},
        "forearm_euler_range_deg": forearm_range.tolist(),
        "hand_euler_range_deg": hand_range.tolist(),
        "pure_wrist_transition_count": int(np.sum(pure_wrist)),
        "pure_wrist_target_step_p95_units": (
            float(np.percentile(pure_wrist_target, 95)) if pure_wrist_target.size else None
        ),
        "pure_wrist_target_step_max_units": (
            float(np.max(pure_wrist_target)) if pure_wrist_target.size else None
        ),
        "checks": {
            "forearm_slot_not_used_as_wrist": bool(float(np.max(forearm_range)) < 1e-3),
            "hand_slot_contains_orientation": bool(float(np.max(hand_range)) > 10.0),
            "pure_wrist_does_not_drive_large_finger_jump": bool(
                pure_wrist_target.size >= 10 and float(np.percentile(pure_wrist_target, 95)) <= 50.0
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = analyze(args.input)
    encoded = json.dumps(report, ensure_ascii=False, indent=2)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    return 0 if all(report["checks"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
