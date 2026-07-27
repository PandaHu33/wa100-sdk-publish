#!/usr/bin/env python3
"""Calibrate time, rotation and wrist-pivot alignment between PICO controller and glove.

Normal runtime keeps owning the original inputs.  It mirrors glove packets to
UDP 25005, while Hand_Tracker mirrors raw right-controller poses to UDP 25006.
This tool only listens to those read-only side channels.
"""

from __future__ import annotations

import argparse
import json
import math
import select
import socket
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from thumb_calibration import (
    conjugate,
    euler_to_quaternion,
    multiply,
    normalize_quaternion,
    quaternion_mean,
)


HI5_FOREARM_INDEX = 0
HI5_HAND_INDEX = 1
HI5_LOCAL_FINGER_START = 2


@dataclass
class PoseSample:
    time_sec: float
    quaternion_wxyz: np.ndarray
    position_m: np.ndarray | None = None
    sequence: int = -1
    raw_rotations_deg: np.ndarray | None = None


def quaternion_angle_deg(value: np.ndarray) -> float:
    q = normalize_quaternion(np.asarray(value, dtype=float))
    return math.degrees(2.0 * math.atan2(float(np.linalg.norm(q[1:])), abs(float(q[0]))))


def quaternion_to_matrix(value: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(np.asarray(value, dtype=float))
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quaternion_left_matrix(value: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(np.asarray(value, dtype=float))
    return np.array(
        [[w, -x, -y, -z], [x, w, -z, y], [y, z, w, -x], [z, -y, x, w]],
        dtype=float,
    )


def quaternion_right_matrix(value: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(np.asarray(value, dtype=float))
    return np.array(
        [[w, -x, -y, -z], [x, w, z, -y], [y, -z, w, x], [z, y, -x, w]],
        dtype=float,
    )


def glove_hand_quaternion(rotations: Sequence[float], euler_order: str) -> np.ndarray:
    values = np.asarray(rotations, dtype=float).reshape(-1)
    if values.size < 63 or not np.all(np.isfinite(values[:63])):
        raise ValueError("rightRotations must contain 63 finite values")
    return euler_to_quaternion(
        values[HI5_HAND_INDEX * 3 : HI5_HAND_INDEX * 3 + 3], euler_order
    )


def glove_proxy_quaternion(rotations: Sequence[float], euler_order: str) -> np.ndarray:
    """Backward-compatible name; the proxy is now the official Hi5 Hand slot."""
    return glove_hand_quaternion(rotations, euler_order)


def angular_speed_series(samples: Sequence[PoseSample]) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    speeds: list[float] = []
    for previous, current in zip(samples, samples[1:]):
        dt = float(current.time_sec - previous.time_sec)
        if dt <= 1e-4 or dt > 0.25:
            continue
        delta = multiply(conjugate(previous.quaternion_wxyz), current.quaternion_wxyz)
        times.append((previous.time_sec + current.time_sec) * 0.5)
        speeds.append(math.radians(quaternion_angle_deg(delta)) / dt)
    return np.asarray(times, dtype=float), np.asarray(speeds, dtype=float)


def estimate_time_offset(
    controller: Sequence[PoseSample],
    glove: Sequence[PoseSample],
    max_offset_sec: float = 0.5,
    step_sec: float = 0.005,
) -> tuple[float, float, int]:
    """Return offset added to controller timestamps to align them to glove time."""
    controller_time, controller_speed = angular_speed_series(controller)
    glove_time, glove_speed = angular_speed_series(glove)
    if min(controller_speed.size, glove_speed.size) < 12:
        raise ValueError("insufficient wrist motion samples for temporal alignment")
    if float(np.std(controller_speed)) < 0.08 or float(np.std(glove_speed)) < 0.08:
        raise ValueError("wrist angular-speed variation is too small")

    best: tuple[float, float, int] | None = None
    offsets = np.arange(-max_offset_sec, max_offset_sec + step_sec * 0.5, step_sec)
    for offset in offsets:
        start = max(float(controller_time[0] + offset), float(glove_time[0]))
        end = min(float(controller_time[-1] + offset), float(glove_time[-1]))
        if end - start < 2.0:
            continue
        grid = np.arange(start, end, 0.01)
        if grid.size < 100:
            continue
        controller_values = np.interp(grid - offset, controller_time, controller_speed)
        glove_values = np.interp(grid, glove_time, glove_speed)
        controller_values -= float(np.mean(controller_values))
        glove_values -= float(np.mean(glove_values))
        denominator = float(np.linalg.norm(controller_values) * np.linalg.norm(glove_values))
        if denominator < 1e-9:
            continue
        correlation = float(np.dot(controller_values, glove_values) / denominator)
        candidate = (correlation, float(offset), int(grid.size))
        if best is None or candidate[0] > best[0]:
            best = candidate
    if best is None:
        raise ValueError("controller/glove streams do not have enough temporal overlap")
    return best[1], best[0], best[2]


def nearest_aligned_pairs(
    controller: Sequence[PoseSample],
    glove: Sequence[PoseSample],
    controller_offset_sec: float,
    max_gap_sec: float = 0.06,
) -> list[tuple[PoseSample, PoseSample]]:
    controller_times = np.asarray([sample.time_sec + controller_offset_sec for sample in controller], dtype=float)
    pairs: list[tuple[PoseSample, PoseSample]] = []
    for glove_sample in glove:
        insertion = int(np.searchsorted(controller_times, glove_sample.time_sec))
        candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(controller)]
        if not candidates:
            continue
        index = min(candidates, key=lambda item: abs(controller_times[item] - glove_sample.time_sec))
        if abs(controller_times[index] - glove_sample.time_sec) <= max_gap_sec:
            pairs.append((controller[index], glove_sample))
    return pairs


def solve_hand_eye_rotation(
    pairs: Sequence[tuple[PoseSample, PoseSample]],
    interval_sec: float = 0.20,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Solve A*X=X*B, where A/B are controller/glove relative rotations."""
    if len(pairs) < 20:
        raise ValueError("insufficient paired orientations for spatial alignment")
    equations: list[np.ndarray] = []
    motions: list[tuple[np.ndarray, np.ndarray]] = []
    axes: list[np.ndarray] = []
    next_index = 1
    for first_index, (controller_first, glove_first) in enumerate(pairs[:-1]):
        next_index = max(next_index, first_index + 1)
        while next_index < len(pairs) and (
            pairs[next_index][1].time_sec - glove_first.time_sec < interval_sec
        ):
            next_index += 1
        if next_index >= len(pairs):
            break
        controller_next, glove_next = pairs[next_index]
        controller_motion = multiply(
            conjugate(controller_first.quaternion_wxyz), controller_next.quaternion_wxyz
        )
        glove_motion = multiply(conjugate(glove_first.quaternion_wxyz), glove_next.quaternion_wxyz)
        controller_angle = quaternion_angle_deg(controller_motion)
        glove_angle = quaternion_angle_deg(glove_motion)
        if min(controller_angle, glove_angle) < 3.0 or max(controller_angle, glove_angle) > 65.0:
            continue
        if abs(controller_angle - glove_angle) > 18.0:
            continue
        equations.append(quaternion_left_matrix(controller_motion) - quaternion_right_matrix(glove_motion))
        motions.append((controller_motion, glove_motion))
        vector = normalize_quaternion(controller_motion)[1:]
        norm = float(np.linalg.norm(vector))
        if norm > 1e-8:
            axes.append(vector / norm)
    if len(equations) < 12:
        raise ValueError("insufficient multi-axis wrist excitation for hand-eye calibration")
    stacked = np.vstack(equations)
    _, _, vh = np.linalg.svd(stacked)
    controller_to_glove = normalize_quaternion(vh[-1])

    residuals = []
    for controller_motion, glove_motion in motions:
        left = multiply(controller_motion, controller_to_glove)
        right = multiply(controller_to_glove, glove_motion)
        residuals.append(quaternion_angle_deg(multiply(conjugate(left), right)))

    world_transforms = [
        multiply(
            glove_sample.quaternion_wxyz,
            conjugate(multiply(controller_sample.quaternion_wxyz, controller_to_glove)),
        )
        for controller_sample, glove_sample in pairs
    ]
    controller_world_to_glove_world = quaternion_mean(world_transforms)
    axis_matrix = np.vstack(axes)
    axis_covariance = axis_matrix.T @ axis_matrix / max(len(axis_matrix), 1)
    eigenvalues = np.sort(np.linalg.eigvalsh(axis_covariance))
    excitation_ratio = float(eigenvalues[0] / max(eigenvalues[-1], 1e-12))
    metrics = {
        "motion_pair_count": len(motions),
        "rotation_residual_median_deg": float(np.median(residuals)),
        "rotation_residual_p95_deg": float(np.percentile(residuals, 95)),
        "axis_excitation_ratio": excitation_ratio,
    }
    return controller_to_glove, controller_world_to_glove_world, metrics


def solve_controller_pivot(controller: Sequence[PoseSample]) -> tuple[np.ndarray, np.ndarray, dict]:
    valid = [sample for sample in controller if sample.position_m is not None]
    if len(valid) < 20:
        raise ValueError("insufficient controller positions for wrist pivot calibration")
    blocks = []
    targets = []
    for sample in valid:
        blocks.append(np.hstack((quaternion_to_matrix(sample.quaternion_wxyz), -np.eye(3))))
        targets.append(-np.asarray(sample.position_m, dtype=float))
    matrix = np.vstack(blocks)
    target = np.hstack(targets)
    solution, _, rank, singular_values = np.linalg.lstsq(matrix, target, rcond=None)
    if rank < 6:
        raise ValueError("controller rotation is insufficient to identify wrist pivot")
    controller_to_pivot = solution[:3]
    pivot_world = solution[3:]
    residuals = np.asarray(
        [
            np.linalg.norm(
                np.asarray(sample.position_m) + quaternion_to_matrix(sample.quaternion_wxyz) @ controller_to_pivot
                - pivot_world
            )
            for sample in valid
        ],
        dtype=float,
    )
    metrics = {
        "pivot_sample_count": len(valid),
        "pivot_residual_median_mm": float(np.median(residuals) * 1000.0),
        "pivot_residual_p95_mm": float(np.percentile(residuals, 95) * 1000.0),
        "pivot_system_condition": float(singular_values[0] / max(singular_values[-1], 1e-12)),
    }
    return controller_to_pivot, pivot_world, metrics


def orientation_spread_deg(samples: Iterable[PoseSample], mean: np.ndarray) -> float:
    values = [
        quaternion_angle_deg(multiply(conjugate(mean), sample.quaternion_wxyz)) for sample in samples
    ]
    return float(np.percentile(values, 95)) if values else float("inf")


class SideChannelCollector:
    def __init__(self, glove_port: int, controller_port: int, euler_order: str):
        self.euler_order = euler_order
        self.glove_socket = self._bind(glove_port)
        self.controller_socket = self._bind(controller_port)
        self.last_glove_sequence: int | None = None
        self.last_controller_sequence: int | None = None

    @staticmethod
    def _bind(port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", int(port)))
        sock.setblocking(False)
        return sock

    def close(self) -> None:
        self.glove_socket.close()
        self.controller_socket.close()

    def collect(self, duration_sec: float, keep: bool = True) -> tuple[list[PoseSample], list[PoseSample]]:
        controller: list[PoseSample] = []
        glove: list[PoseSample] = []
        deadline = time.perf_counter() + max(float(duration_sec), 0.0)
        while time.perf_counter() < deadline:
            ready, _, _ = select.select(
                [self.glove_socket, self.controller_socket], [], [], min(0.1, deadline - time.perf_counter())
            )
            for sock in ready:
                try:
                    raw, _ = sock.recvfrom(65535)
                    payload = json.loads(raw.decode("utf-8"))
                    arrival = time.time()
                except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if sock is self.controller_socket:
                    sample = self._controller_sample(payload, arrival)
                    if sample is not None and keep:
                        controller.append(sample)
                else:
                    sample = self._glove_sample(payload, arrival)
                    if sample is not None and keep:
                        glove.append(sample)
        return controller, glove

    def _controller_sample(self, payload: dict, arrival: float) -> PoseSample | None:
        if payload.get("type") != "controller_pose_frame" or payload.get("source") != "right_controller":
            return None
        if not bool(payload.get("tracked", False)):
            return None
        try:
            sequence = int(payload["seq"])
            xyzw = np.asarray(payload["rotation_xyzw"], dtype=float)
            position = np.asarray(payload["position_m"], dtype=float)
            if xyzw.shape != (4,) or position.shape != (3,):
                return None
            quaternion = normalize_quaternion(np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]]))
        except (KeyError, TypeError, ValueError):
            return None
        if sequence == self.last_controller_sequence:
            return None
        self.last_controller_sequence = sequence
        source_time = float(payload.get("receiver_wall_time_ns", 0)) / 1e9
        return PoseSample(source_time if source_time > 0 else arrival, quaternion, position, sequence)

    def _glove_sample(self, payload: dict, arrival: float) -> PoseSample | None:
        rotations = payload.get("rightRotations")
        if not isinstance(rotations, list) or len(rotations) != 63:
            return None
        try:
            sequence = int(payload.get("frameId", -1))
            quaternion = glove_hand_quaternion(rotations, self.euler_order)
        except (TypeError, ValueError):
            return None
        if sequence >= 0 and sequence == self.last_glove_sequence:
            return None
        self.last_glove_sequence = sequence
        source_time = float(payload.get("receiver_wall_time_ns", 0)) / 1e9
        raw_values = np.asarray(rotations, dtype=float).reshape(21, 3)
        return PoseSample(
            source_time if source_time > 0 else arrival,
            quaternion,
            None,
            sequence,
            raw_values,
        )


def save_raw_samples(
    path: Path,
    neutral_controller: Sequence[PoseSample],
    neutral_glove: Sequence[PoseSample],
    motion_controller: Sequence[PoseSample],
    motion_glove: Sequence[PoseSample],
) -> None:
    def times(samples: Sequence[PoseSample]) -> np.ndarray:
        return np.asarray([sample.time_sec for sample in samples], dtype=float)

    def quaternions(samples: Sequence[PoseSample]) -> np.ndarray:
        return np.vstack([sample.quaternion_wxyz for sample in samples])

    def positions(samples: Sequence[PoseSample]) -> np.ndarray:
        return np.vstack([sample.position_m for sample in samples])

    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        neutral_controller_time=times(neutral_controller),
        neutral_controller_quaternion=quaternions(neutral_controller),
        neutral_controller_position=positions(neutral_controller),
        neutral_glove_time=times(neutral_glove),
        neutral_glove_quaternion=quaternions(neutral_glove),
        motion_controller_time=times(motion_controller),
        motion_controller_quaternion=quaternions(motion_controller),
        motion_controller_position=positions(motion_controller),
        motion_glove_time=times(motion_glove),
        motion_glove_quaternion=quaternions(motion_glove),
        neutral_glove_rotations=np.stack([sample.raw_rotations_deg for sample in neutral_glove]),
        motion_glove_rotations=np.stack([sample.raw_rotations_deg for sample in motion_glove]),
    )


def load_raw_samples(
    path: Path, euler_order: str
) -> tuple[list[PoseSample], list[PoseSample], list[PoseSample], list[PoseSample]]:
    """Reload NPZ samples and reconstruct the glove orientation from raw Hand=1."""
    with np.load(path) as archive:
        required_raw = {"neutral_glove_rotations", "motion_glove_rotations"}
        missing_raw = sorted(required_raw.difference(archive.files))
        if missing_raw:
            raise ValueError(
                "legacy archive cannot be re-fit safely because it lacks raw Hand=1 rotations: "
                + ", ".join(missing_raw)
            )

        def load_controller(prefix: str) -> list[PoseSample]:
            return [
                PoseSample(float(t), normalize_quaternion(q), np.asarray(p, dtype=float), index)
                for index, (t, q, p) in enumerate(
                    zip(
                        archive[f"{prefix}_controller_time"],
                        archive[f"{prefix}_controller_quaternion"],
                        archive[f"{prefix}_controller_position"],
                    )
                )
            ]

        def load_glove(prefix: str) -> list[PoseSample]:
            return [
                PoseSample(
                    float(t),
                    glove_hand_quaternion(frame.reshape(-1), euler_order),
                    None,
                    index,
                    np.asarray(frame, dtype=float),
                )
                for index, (t, frame) in enumerate(
                    zip(
                        archive[f"{prefix}_glove_time"],
                        archive[f"{prefix}_glove_rotations"],
                    )
                )
            ]

        return (
            load_controller("neutral"),
            load_glove("neutral"),
            load_controller("motion"),
            load_glove("motion"),
        )


def fit_alignment(
    neutral_controller: Sequence[PoseSample],
    neutral_glove: Sequence[PoseSample],
    motion_controller: Sequence[PoseSample],
    motion_glove: Sequence[PoseSample],
    euler_order: str,
    require_pivot: bool = False,
) -> dict:
    if min(len(neutral_controller), len(neutral_glove)) < 20:
        raise ValueError("neutral stage has fewer than 20 valid frames in one stream")
    controller_offset, time_correlation, correlation_samples = estimate_time_offset(
        motion_controller, motion_glove
    )
    pairs = nearest_aligned_pairs(motion_controller, motion_glove, controller_offset)
    if len(pairs) < 60:
        raise ValueError("fewer than 60 paired frames; training/validation split is not reliable")
    pair_gaps_ms = []
    for controller_sample, glove_sample in pairs:
        pair_gaps_ms.append(
            abs(controller_sample.time_sec + controller_offset - glove_sample.time_sec) * 1000.0
        )

    neutral_controller_mean = quaternion_mean(sample.quaternion_wxyz for sample in neutral_controller)
    neutral_glove_mean = quaternion_mean(sample.quaternion_wxyz for sample in neutral_glove)
    motion_controller_mean = quaternion_mean(sample.quaternion_wxyz for sample in motion_controller)
    motion_glove_mean = quaternion_mean(sample.quaternion_wxyz for sample in motion_glove)
    controller_motion_p95 = orientation_spread_deg(motion_controller, motion_controller_mean)
    glove_motion_p95 = orientation_spread_deg(motion_glove, motion_glove_mean)
    split_index = max(20, min(len(pairs) - 20, int(len(pairs) * 0.75)))
    train_pairs, validation_pairs = pairs[:split_index], pairs[split_index:]
    controller_to_glove, controller_world_to_glove_world, hand_eye_metrics = solve_hand_eye_rotation(
        train_pairs
    )
    validation_residuals = []
    for controller_sample, glove_sample in validation_pairs:
        predicted = multiply(
            multiply(controller_world_to_glove_world, controller_sample.quaternion_wxyz),
            controller_to_glove,
        )
        validation_residuals.append(
            quaternion_angle_deg(multiply(conjugate(predicted), glove_sample.quaternion_wxyz))
        )
    if len(validation_residuals) < 10:
        raise ValueError("independent validation segment has fewer than 10 paired orientations")
    rotation_metrics = {
        "rotation_model": "rigid_hand_eye",
        "controller_motion_p95_deg": controller_motion_p95,
        "glove_hand_motion_p95_deg": glove_motion_p95,
        "train_pair_count": len(train_pairs),
        "validation_pair_count": len(validation_pairs),
        "validation_rotation_residual_median_deg": float(np.median(validation_residuals)),
        "validation_rotation_residual_p95_deg": float(np.percentile(validation_residuals, 95)),
        **hand_eye_metrics,
    }
    rotation_alignment = {
        "mode": "rigid_hand_eye",
        "equation": "R_hand = R_gloveWorld_from_controllerWorld * R_controller * R_controller_to_hand",
        "controller_to_glove_hand_quaternion_wxyz": controller_to_glove.tolist(),
        "glove_world_from_controller_world_quaternion_wxyz": controller_world_to_glove_world.tolist(),
        "neutral_controller_quaternion_wxyz": neutral_controller_mean.tolist(),
        "neutral_glove_hand_quaternion_wxyz": neutral_glove_mean.tolist(),
    }

    neutral_raw = [sample.raw_rotations_deg for sample in neutral_glove if sample.raw_rotations_deg is not None]
    if neutral_raw:
        stacked_raw = np.stack(neutral_raw)
        rotation_alignment["neutral_glove_joint_euler_deg"] = np.median(stacked_raw, axis=0).tolist()
        rotation_alignment["neutral_glove_joint_quaternions_wxyz"] = [
            quaternion_mean(
                euler_to_quaternion(frame[joint_index], euler_order) for frame in stacked_raw
            ).tolist()
            for joint_index in range(21)
        ]

    try:
        controller_to_pivot, pivot_world, pivot_metrics = solve_controller_pivot(motion_controller)
        pivot_error = ""
    except (ValueError, np.linalg.LinAlgError) as exc:
        controller_to_pivot = np.zeros(3, dtype=float)
        pivot_world = np.zeros(3, dtype=float)
        pivot_metrics = {
            "pivot_sample_count": len(motion_controller),
            "pivot_residual_median_mm": None,
            "pivot_residual_p95_mm": None,
            "pivot_system_condition": None,
        }
        pivot_error = str(exc)

    neutral_positions = np.vstack([sample.position_m for sample in neutral_controller])
    neutral_metrics = {
        "controller_orientation_p95_deg": orientation_spread_deg(
            neutral_controller, neutral_controller_mean
        ),
        "glove_orientation_p95_deg": orientation_spread_deg(neutral_glove, neutral_glove_mean),
        "controller_position_p95_mm": float(
            np.percentile(np.linalg.norm(neutral_positions - np.median(neutral_positions, axis=0), axis=1), 95)
            * 1000.0
        ),
    }
    metrics = {
        "time_alignment_method": "angular_speed_cross_correlation_on_common_pc_clock",
        "time_correlation_coefficient": time_correlation,
        "time_correlation_sample_count": correlation_samples,
        "paired_time_gap_median_ms": float(np.median(pair_gaps_ms)),
        "paired_time_gap_p95_ms": float(np.percentile(pair_gaps_ms, 95)),
        "paired_orientation_count": len(pairs),
        **rotation_metrics,
        **pivot_metrics,
        **neutral_metrics,
    }
    time_pass = (
        len(pairs) >= 50
        and time_correlation >= 0.8
        and metrics["paired_time_gap_p95_ms"] <= 60.0
    )
    rotation_pass = (
        rotation_metrics["motion_pair_count"] >= 12
        and rotation_metrics["validation_rotation_residual_median_deg"] <= 4.0
        and rotation_metrics["validation_rotation_residual_p95_deg"] <= 10.0
        and rotation_metrics["axis_excitation_ratio"] >= 0.03
    )
    pivot_p95 = pivot_metrics.get("pivot_residual_p95_mm")
    pivot_pass = bool(pivot_p95 is not None and pivot_p95 <= 30.0)
    neutral_pass = (
        neutral_metrics["controller_orientation_p95_deg"] <= 3.0
        and neutral_metrics["glove_orientation_p95_deg"] <= 4.0
        and neutral_metrics["controller_position_p95_mm"] <= 8.0
    )
    metrics.update(
        {
            "time_pass": time_pass,
            "rotation_pass": rotation_pass,
            "pivot_pass": pivot_pass,
            "neutral_pass": neutral_pass,
            "pivot_required": bool(require_pivot),
            "quality_pass": bool(
                time_pass and rotation_pass and (pivot_pass or not require_pivot)
            ),
        }
    )
    return {
        "schema_version": 2,
        "calibration_id": "glove-controller-" + datetime.now().strftime("%Y%m%d-%H%M%S"),
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "hand": "right",
        "controller_source": "right_controller",
        "glove_source": "rightRotations",
        "conventions": {
            "controller_quaternion_input": "xyzw",
            "stored_quaternions": "wxyz",
            "glove_euler_order": euler_order,
            "hi5_bone_layout": {
                "0": "ForeArm",
                "1": "Hand",
                "2-20": "local_finger_bones",
            },
            "glove_orientation_index": HI5_HAND_INDEX,
            "glove_orientation_name": "Hand",
        },
        "time_alignment": {
            "controller_time_offset_to_glove_sec": controller_offset,
            "correlation_coefficient": time_correlation,
            "method": "angular_speed_cross_correlation_on_common_pc_receive_clock",
            "meaning": "Add this offset to controller PC-receive timestamps before pairing with glove frames",
        },
        "rotation_alignment": rotation_alignment,
        "position_alignment": {
            "method": "controller_pivot_calibration",
            "required": bool(require_pivot),
            "valid": pivot_pass,
            "error": pivot_error,
            "controller_to_wrist_pivot_m": controller_to_pivot.tolist(),
            "estimated_world_pivot_m": pivot_world.tolist(),
        },
        "sample_counts": {
            "neutral_controller": len(neutral_controller),
            "neutral_glove": len(neutral_glove),
            "motion_controller": len(motion_controller),
            "motion_glove": len(motion_glove),
        },
        "quality": metrics,
    }


def main() -> int:
    default_output = Path(__file__).resolve().parents[1] / "build" / "bin" / "glove_controller_alignment.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--glove-port", type=int, default=25005)
    parser.add_argument("--controller-port", type=int, default=25006)
    parser.add_argument("--euler-order", default="zyx", choices=("xyz", "xzy", "yxz", "yzx", "zxy", "zyx"))
    parser.add_argument("--neutral-duration", type=float, default=3.0)
    parser.add_argument("--motion-duration", type=float, default=12.0)
    parser.add_argument("--stage-delay", type=float, default=5.0)
    parser.add_argument("--stream-timeout", type=float, default=90.0)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--samples", type=Path)
    parser.add_argument(
        "--require-pivot",
        action="store_true",
        help="Require <=30 mm pivot P95. Off by default for 5-DoF incremental control.",
    )
    args = parser.parse_args()
    samples_path = args.samples or args.output.with_name("glove_controller_alignment_samples.npz")

    collector = SideChannelCollector(args.glove_port, args.controller_port, args.euler_order)
    try:
        print(f"等待数据：手套 UDP {args.glove_port}，手柄 UDP {args.controller_port}", flush=True)
        deadline = time.perf_counter() + args.stream_timeout
        seen_controller = seen_glove = False
        while time.perf_counter() < deadline and not (seen_controller and seen_glove):
            controller, glove = collector.collect(0.5)
            seen_controller = seen_controller or bool(controller)
            seen_glove = seen_glove or bool(glove)
            print(
                f"\r手柄={'就绪' if seen_controller else '等待'}  手套={'就绪' if seen_glove else '等待'}",
                end="",
                flush=True,
            )
        print()
        if not (seen_controller and seen_glove):
            raise RuntimeError("等待数据超时；请确认显控、PICO、Unity 和 UDP 接收器均已启动")

        print(f"{args.stage_delay:.0f} 秒后采集中立姿态：手腕伸直、五指自然张开并保持静止。", flush=True)
        collector.collect(args.stage_delay, keep=False)
        neutral_controller, neutral_glove = collector.collect(args.neutral_duration)
        print(f"中立采集完成：手柄 {len(neutral_controller)} 帧，手套 {len(neutral_glove)} 帧", flush=True)

        print(
            f"{args.stage_delay:.0f} 秒后采集腕部运动：前臂尽量固定、手指不动，围绕腕心缓慢做三轴旋转。",
            flush=True,
        )
        collector.collect(args.stage_delay, keep=False)
        motion_controller, motion_glove = collector.collect(args.motion_duration)
        print(f"运动采集完成：手柄 {len(motion_controller)} 帧，手套 {len(motion_glove)} 帧", flush=True)
    finally:
        collector.close()

    save_raw_samples(
        samples_path, neutral_controller, neutral_glove, motion_controller, motion_glove
    )
    print(f"原始样本：{samples_path.resolve()}")
    payload = fit_alignment(
        neutral_controller,
        neutral_glove,
        motion_controller,
        motion_glove,
        args.euler_order,
        require_pivot=args.require_pivot,
    )
    print(json.dumps(payload["quality"], ensure_ascii=False, indent=2))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not payload["quality"]["quality_pass"]:
        failed_path = args.output.with_name(args.output.stem + ".failed.json")
        failed_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"标定未通过，不覆盖运行配置。诊断：{failed_path.resolve()}", file=sys.stderr)
        return 2
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"标定通过：{args.output.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n标定已取消", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"标定失败：{exc}", file=sys.stderr)
        raise SystemExit(1)
