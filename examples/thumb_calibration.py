#!/usr/bin/env python3
"""Guided rotation-only calibration for the Hi5 glove thumb.

Stop udp_receiver_unity.exe before running this program so it can own UDP 25001.
The Unity/Hi5 sender may remain running. A successful calibration is written next
to the deployed WA100 executable by default.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np


UDP_IP = "127.0.0.1"
UDP_PORT = 25001
BUFFER_SIZE = 65536
JOINT_INDICES = (1, 2, 3)
EULER_ORDERS = ("xyz", "xzy", "yxz", "yzx", "zxy", "zyx")
FEATURE_DIMENSION = len(JOINT_INDICES) * 3


@dataclass(frozen=True)
class State:
    key: str
    label: str
    instruction: str
    duration: float


STATES = (
    State("neutral", "中立静止", "固定手腕，拇指自然外展并伸直；之后所有阶段保持同一腕姿。", 3.0),
    State("yaw_open", "拇指横摇张开端", "手腕固定，拇指保持伸直，横向自然外展并保持。", 3.0),
    State("yaw_close", "拇指横摇对掌端", "手腕固定，拇指保持伸直，横向移向掌心并保持。", 3.0),
    State("yaw_sweep", "拇指横摇往返", "手腕固定、拇指保持伸直，横摇缓慢往返至少三次。", 10.0),
    State("pitch_open", "拇指弯曲张开端", "手腕固定，拇指保持自然横摇位置并完全伸直。", 3.0),
    State("pitch_close", "拇指弯曲闭合端", "手腕固定、横摇位置不变，只让拇指完全弯曲并保持。", 3.0),
    State("pitch_sweep", "拇指弯曲往返", "手腕固定、横摇位置不变，弯曲和伸直缓慢往返至少三次。", 10.0),
    State("combined", "拇指组合动作", "手腕固定，组合进行横摇和弯曲，覆盖日常使用范围。", 10.0),
    State("neutral_validate", "中立姿态复测", "保持同一腕姿，恢复拇指自然外展和伸直并保持。", 5.0),
)


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    result = q / norm
    return -result if result[0] < 0.0 else result


def multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w, x, y, z = left
    rw, rx, ry, rz = right
    return np.array(
        [
            w * rw - x * rx - y * ry - z * rz,
            w * rx + x * rw + y * rz - z * ry,
            w * ry - x * rz + y * rw + z * rx,
            w * rz + x * ry - y * rx + z * rw,
        ],
        dtype=float,
    )


def axis_quaternion(axis: str, angle_deg: float) -> np.ndarray:
    half = math.radians(angle_deg) * 0.5
    result = np.array([math.cos(half), 0.0, 0.0, 0.0], dtype=float)
    result["xyz".index(axis) + 1] = math.sin(half)
    return result


def euler_to_quaternion(euler: Sequence[float], order: str) -> np.ndarray:
    result = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    values = dict(zip("xyz", euler))
    for axis in order:
        result = multiply(result, axis_quaternion(axis, float(values[axis])))
    return normalize_quaternion(result)


def quaternion_log_deg(q: np.ndarray) -> np.ndarray:
    value = normalize_quaternion(q)
    vector_norm = float(np.linalg.norm(value[1:]))
    if vector_norm < 1e-12:
        return np.zeros(3, dtype=float)
    angle = math.degrees(2.0 * math.atan2(vector_norm, float(np.clip(value[0], -1.0, 1.0))))
    return value[1:] * (angle / vector_norm)


def quaternion_mean(values: Iterable[np.ndarray]) -> np.ndarray:
    aligned: List[np.ndarray] = []
    for value in values:
        q = normalize_quaternion(np.asarray(value, dtype=float))
        if aligned and float(np.dot(q, aligned[0])) < 0.0:
            q = -q
        aligned.append(q)
    if not aligned:
        raise ValueError("cannot average an empty quaternion series")
    return normalize_quaternion(np.mean(np.vstack(aligned), axis=0))


def extract_selected_rotations(packet: dict, hand: str) -> np.ndarray:
    key = f"{hand}Rotations"
    values = packet.get(key)
    if not isinstance(values, list) or len(values) < 63:
        raise ValueError(f"missing or short {key}")
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
        raise ValueError(f"{key} contains invalid values")
    return np.asarray([values[index * 3 : index * 3 + 3] for index in JOINT_INDICES], dtype=float)


def receive_packet(sock: socket.socket, hand: str) -> Tuple[int, np.ndarray]:
    while True:
        data, _ = sock.recvfrom(BUFFER_SIZE)
        try:
            packet = json.loads(data.decode("utf-8"))
            return int(packet.get("frameId", -1)), extract_selected_rotations(packet, hand)
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
            continue


def collect_state(sock: socket.socket, hand: str, state: State, duration_override: float | None) -> List[np.ndarray]:
    duration = duration_override if duration_override is not None else state.duration
    print(f"\n[{state.label}] {state.instruction}")
    input(f"准备好后按回车，采集 {duration:g} 秒...")
    deadline = time.monotonic() + duration
    frames: List[np.ndarray] = []
    last_frame = None
    sock.settimeout(0.5)
    while time.monotonic() < deadline:
        try:
            frame_id, rotations = receive_packet(sock, hand)
        except socket.timeout:
            continue
        if frame_id >= 0 and last_frame is not None and frame_id <= last_frame:
            continue
        last_frame = frame_id
        frames.append(rotations)
    print(f"完成，共 {len(frames)} 帧")
    if len(frames) < max(20, int(duration * 10)):
        raise RuntimeError(f"{state.label} 有效帧不足，请确认 Unity/Hi5 正在向 UDP {UDP_PORT} 发送")
    return frames


def build_references(neutral: Sequence[np.ndarray], order: str) -> np.ndarray:
    references = []
    for selected in range(len(JOINT_INDICES)):
        references.append(quaternion_mean(euler_to_quaternion(sample[selected], order) for sample in neutral))
    return np.vstack(references)


def conjugate(value: np.ndarray) -> np.ndarray:
    return np.array([value[0], -value[1], -value[2], -value[3]], dtype=float)


def features(samples: Sequence[np.ndarray], references: np.ndarray, order: str) -> np.ndarray:
    rows = []
    for sample in samples:
        row = []
        for selected in range(len(JOINT_INDICES)):
            current = euler_to_quaternion(sample[selected], order)
            # The Hi5 packet exposes three thumb-bone rotations even when the
            # glove has one physical thumb sensor. Preserve all three derived
            # slots so calibration can determine whether two motions remain
            # observable; do not introduce a wrist reference in this baseline.
            delta = multiply(conjugate(references[selected]), current)
            row.extend(quaternion_log_deg(delta))
        rows.append(row)
    return np.asarray(rows, dtype=float)


def target_ratio(score: np.ndarray, open_score: float, close_score: float) -> np.ndarray:
    ratio = np.clip((score - open_score) / (close_score - open_score), 0.0, 1.0)
    output = np.empty_like(ratio)
    low, high = 0.04, 0.94
    output[ratio <= low] = 0.0
    output[ratio >= high] = 1.0
    middle_mask = (ratio > low) & (ratio < high)
    middle = (ratio[middle_mask] - low) / (high - low)
    output[middle_mask] = middle * middle * (3.0 - 2.0 * middle)
    return output


def target_range(score: np.ndarray, open_score: float, close_score: float) -> float:
    return float(np.ptp(2000.0 * (1.0 - target_ratio(score, open_score, close_score))))


def thumb_pitch_magnitude(samples: List[np.ndarray]) -> np.ndarray:
    """Use the single thumb sensor's derived MCP rotation magnitude."""
    values = np.asarray(samples, dtype=float)
    if values.ndim != 3 or values.shape[1:] != (len(JOINT_INDICES), 3):
        raise ValueError("thumb pitch samples must have shape [N, 3, 3]")
    return np.linalg.norm(values[:, 1, :], axis=1)


def fit_candidate(samples: Dict[str, List[np.ndarray]], order: str) -> dict:
    references = build_references(samples["neutral"], order)
    matrices = {key: features(value, references, order) for key, value in samples.items()}
    yaw_delta = np.mean(matrices["yaw_close"], axis=0) - np.mean(matrices["yaw_open"], axis=0)
    pitch_delta = np.mean(matrices["pitch_close"], axis=0) - np.mean(matrices["pitch_open"], axis=0)
    yaw_norm, pitch_norm = float(np.linalg.norm(yaw_delta)), float(np.linalg.norm(pitch_delta))
    if min(yaw_norm, pitch_norm) < 1e-6:
        raise ValueError(f"{order}: thumb endpoint motion is degenerate")
    neutral_covariance = np.cov(np.vstack((matrices["neutral"], matrices["neutral_validate"])), rowvar=False)
    yaw_covariance = np.cov(matrices["yaw_sweep"], rowvar=False)
    pitch_covariance = np.cov(matrices["pitch_sweep"], rowvar=False)
    scale = max(float(np.trace(neutral_covariance + yaw_covariance + pitch_covariance)) / (3.0 * FEATURE_DIMENSION), 1e-8)
    identity = np.eye(FEATURE_DIMENSION)
    pitch_magnitudes = {key: thumb_pitch_magnitude(value) for key, value in samples.items()}
    pitch_open_magnitude = float(np.median(pitch_magnitudes["pitch_open"]))
    pitch_close_magnitude = float(np.median(pitch_magnitudes["pitch_close"]))
    pitch_magnitude_span = abs(pitch_close_magnitude - pitch_open_magnitude)
    if pitch_magnitude_span < 1e-6:
        raise ValueError(f"{order}: thumb pitch rotation magnitude is degenerate")
    candidates = []

    # Fisher-style directions use the continuous sweeps as negative examples:
    # yaw must respond to yaw endpoints while remaining quiet during pitch, and
    # vice versa. This avoids an unstable endpoint-matrix inversion for the
    # single physical sensor's derived Hi5 bone rotations.
    for neutral_weight in (0.1, 0.3, 1.0, 3.0, 10.0):
        for ridge_ratio in (0.0001, 0.0003, 0.001, 0.003, 0.01):
            yaw_axis = np.linalg.solve(
                pitch_covariance + neutral_weight * neutral_covariance + ridge_ratio * scale * identity,
                yaw_delta,
            )
            pitch_axis = np.linalg.solve(
                yaw_covariance + neutral_weight * neutral_covariance + ridge_ratio * scale * identity,
                pitch_delta,
            )
            yaw_axis /= max(float(np.linalg.norm(yaw_axis)), 1e-12)
            pitch_axis /= max(float(np.linalg.norm(pitch_axis)), 1e-12)
            scores = {
                key: np.column_stack((value @ yaw_axis, value @ pitch_axis))
                for key, value in matrices.items()
            }
            yaw_change = np.mean(scores["yaw_close"], axis=0) - np.mean(scores["yaw_open"], axis=0)
            pitch_change = np.mean(scores["pitch_close"], axis=0) - np.mean(scores["pitch_open"], axis=0)
            response = np.column_stack((yaw_change, pitch_change))
            condition = float(np.linalg.cond(response))
            if not math.isfinite(condition) or condition > 1000.0:
                continue
            yaw_open = float(np.median(scores["yaw_open"][:, 0]))
            yaw_close = float(np.median(scores["yaw_close"][:, 0]))
            pitch_open = pitch_open_magnitude
            pitch_close = pitch_close_magnitude
            metrics = {
                "response_condition": condition,
                "yaw_feature_span": yaw_norm,
                "pitch_feature_span": pitch_magnitude_span,
                "discriminant_neutral_weight": neutral_weight,
                "discriminant_ridge_ratio": ridge_ratio,
                "neutral_validation_yaw_target_range": target_range(scores["neutral_validate"][:, 0], yaw_open, yaw_close),
                "neutral_validation_pitch_target_range": target_range(
                    pitch_magnitudes["neutral_validate"], pitch_open, pitch_close),
                "yaw_sweep_target_range": target_range(scores["yaw_sweep"][:, 0], yaw_open, yaw_close),
                "yaw_sweep_pitch_crosstalk": target_range(
                    pitch_magnitudes["yaw_sweep"], pitch_open, pitch_close),
                "pitch_sweep_target_range": target_range(
                    pitch_magnitudes["pitch_sweep"], pitch_open, pitch_close),
                "pitch_sweep_yaw_crosstalk": target_range(scores["pitch_sweep"][:, 0], yaw_open, yaw_close),
            }
            metrics["quality_pass"] = bool(
                yaw_norm >= 5.0
                and pitch_magnitude_span >= 5.0
                and condition <= 20.0
                and metrics["neutral_validation_yaw_target_range"] <= 100.0
                and metrics["neutral_validation_pitch_target_range"] <= 100.0
                and metrics["yaw_sweep_target_range"] >= 1600.0
                and metrics["pitch_sweep_target_range"] >= 1600.0
                and metrics["pitch_sweep_yaw_crosstalk"] <= 100.0
            )
            penalty = (
                metrics["neutral_validation_yaw_target_range"]
                + metrics["neutral_validation_pitch_target_range"]
                + metrics["yaw_sweep_pitch_crosstalk"]
                + metrics["pitch_sweep_yaw_crosstalk"]
            )
            reward = metrics["yaw_sweep_target_range"] + metrics["pitch_sweep_target_range"]
            selection_score = reward - 4.0 * penalty - 10.0 * condition
            candidates.append({
                "order": order,
                "references": references,
                "yaw_axis": yaw_axis,
                "pitch_axis": pitch_axis,
                "demix": np.eye(2),
                "ranges": {"yaw": {"open": yaw_open, "close": yaw_close}, "pitch": {"open": pitch_open, "close": pitch_close}},
                "metrics": metrics,
                "selection_score": float(selection_score),
            })
    if not candidates:
        raise ValueError(f"{order}: no stable discriminant solution")
    candidates.sort(key=lambda item: (item["metrics"]["quality_pass"], item["selection_score"]), reverse=True)
    return candidates[0]


def calibration_payload(hand: str, fit: dict, sample_counts: Dict[str, int]) -> dict:
    calibration_id = "hi5-thumb-" + datetime.now().strftime("%Y%m%d-%H%M%S")
    return {
        "schema_version": 2,
        "calibration_id": calibration_id,
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "hand": hand,
        "source": "hi5_single_sensor_derived_thumb_rotations_euler_deg",
        "model": "single_thumb_fixed_wrist",
        "euler_order": fit["order"],
        "feature_joint_indices": list(JOINT_INDICES),
        "feature_joint_names": ["thumb_cmc_derived", "thumb_mcp_derived", "thumb_ip_derived"],
        "reference_quaternions_wxyz": fit["references"].tolist(),
        "yaw_axis": fit["yaw_axis"].tolist(),
        "pitch_axis": fit["pitch_axis"].tolist(),
        "demix_matrix": fit["demix"].tolist(),
        "score_ranges": fit["ranges"],
        "pitch_feature": {
            "model": "single_sensor_euler_rotation_magnitude",
            "joint": "thumb_mcp_derived",
            "unit": "degree",
        },
        "mapping": {"open_target": 2000, "close_target": 0, "open_deadzone": 0.04, "close_deadzone": 0.06},
        "quality": fit["metrics"],
        "sample_counts": sample_counts,
    }


def save_samples(path: Path, samples: Dict[str, List[np.ndarray]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **{key: np.stack(value) for key, value in samples.items()})


def main() -> int:
    default_output = Path(__file__).resolve().parents[1] / "build" / "bin" / "thumb_glove_calibration.json"
    parser = argparse.ArgumentParser(description="Hi5 glove rotation-only thumb calibration")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--ip", default=UDP_IP)
    parser.add_argument("--port", type=int, default=UDP_PORT)
    parser.add_argument("--duration", type=float, help="override every state duration for a quick dry run")
    parser.add_argument("--output", default=str(default_output))
    parser.add_argument("--diagnostics", help="diagnostic JSON path; defaults next to output")
    parser.add_argument("--samples", help="raw NPZ sample path; defaults next to output")
    parser.add_argument("--from-samples", type=Path, help="refit an existing raw NPZ without collecting UDP data")
    args = parser.parse_args()

    output = Path(args.output).resolve()
    diagnostics = Path(args.diagnostics).resolve() if args.diagnostics else output.with_name("thumb_glove_calibration_diagnostics.json")
    samples_path = Path(args.samples).resolve() if args.samples else output.with_name("thumb_glove_calibration_samples.npz")
    print("=== Hi5 右手套拇指 rotation-only 标定 ===")
    if args.from_samples:
        with np.load(args.from_samples.resolve()) as archive:
            samples = {key: [frame for frame in archive[key]] for key in archive.files}
        missing = [state.key for state in STATES if state.key not in samples]
        if missing:
            raise RuntimeError("原始样本缺少阶段: " + ", ".join(missing))
        print(f"重新拟合原始样本: {args.from_samples.resolve()}")
    else:
        print("请先停止 udp_receiver_unity.exe；Unity/Hi5 发送程序保持运行。")
        print(f"监听 {args.ip}:{args.port}，手侧={args.hand}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((args.ip, args.port))
        samples = {}
        try:
            for state in STATES:
                samples[state.key] = collect_state(sock, args.hand, state, args.duration)
        finally:
            sock.close()
        save_samples(samples_path, samples)
        print(f"原始样本: {samples_path}")

    candidates = []
    failures = []
    for order in EULER_ORDERS:
        try:
            candidates.append(fit_candidate(samples, order))
        except (ValueError, np.linalg.LinAlgError) as exc:
            failures.append(str(exc))
    if not candidates:
        raise RuntimeError("所有欧拉角顺序均拟合失败：" + "; ".join(failures))
    candidates.sort(key=lambda item: (item["metrics"]["quality_pass"], item["selection_score"]), reverse=True)
    best = candidates[0]
    payload = calibration_payload(args.hand, best, {key: len(value) for key, value in samples.items()})
    diagnostic_payload = {
        "selected_order": best["order"],
        "quality_pass": best["metrics"]["quality_pass"],
        "candidates": [
            {"order": item["order"], "selection_score": item["selection_score"], "metrics": item["metrics"]}
            for item in candidates
        ],
        "failures": failures,
    }
    diagnostics.parent.mkdir(parents=True, exist_ok=True)
    diagnostics.write_text(json.dumps(diagnostic_payload, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"\n选择欧拉角顺序: {best['order']}")
    print(json.dumps(best["metrics"], ensure_ascii=False, indent=2))
    print(f"诊断结果: {diagnostics}")
    if not best["metrics"]["quality_pass"]:
        print("\n标定未通过质量门限，未写入运行时配置。请按诊断结果重新采集。", file=sys.stderr)
        return 2

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n标定通过，运行时配置已写入: {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n标定已中断", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"标定失败: {exc}", file=sys.stderr)
        raise SystemExit(1)
