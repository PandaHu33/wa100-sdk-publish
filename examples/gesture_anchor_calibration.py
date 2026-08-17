#!/usr/bin/env python3
"""标定食指碰拇指、中指碰拇指两个 Hand 局部指尖锚点。"""

from __future__ import annotations

import argparse
import json
import socket
import time
from datetime import datetime
from pathlib import Path

import numpy as np


PARENTS = (-1, 0, 1, 2, 3, 1, 5, 6, 7, 1, 9, 10, 11, 1, 13, 14, 15, 1, 17, 18, 19)
TIP_INDICES = (4, 8, 12)
STAGES = (
    ("thumb_index_touch", "食指指尖与拇指指尖相碰，其余手指自然保持"),
    ("thumb_middle_touch", "中指指尖与拇指指尖相碰，其余手指自然保持"),
)


def multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w, x, y, z = left
    rw, rx, ry, rz = right
    return np.array([
        w * rw - x * rx - y * ry - z * rz,
        w * rx + x * rw + y * rz - z * ry,
        w * ry - x * rz + y * rw + z * rx,
        w * rz + x * ry - y * rx + z * rw,
    ])


def euler_quaternion(value: np.ndarray, order: str) -> np.ndarray:
    result = np.array([1.0, 0.0, 0.0, 0.0])
    for axis in order:
        angle = np.deg2rad(value["xyz".index(axis)]) * 0.5
        item = np.array([np.cos(angle), 0.0, 0.0, 0.0])
        item[1 + "xyz".index(axis)] = np.sin(angle)
        result = multiply(result, item)
    return result / max(np.linalg.norm(result), 1e-12)


def rotate(q: np.ndarray, value: np.ndarray) -> np.ndarray:
    vector = np.r_[0.0, value]
    conjugate = q * np.array([1.0, -1.0, -1.0, -1.0])
    return multiply(multiply(q, vector), conjugate)[1:]


def tip_feature(payload: dict, order: str, hand: str) -> np.ndarray:
    prefix = "right" if hand == "right" else "left"
    offsets = np.asarray(payload[f"{prefix}Positions"], dtype=float).reshape(21, 3)
    rotations = np.asarray(payload[f"{prefix}Rotations"], dtype=float).reshape(21, 3)
    positions = np.zeros((21, 3), dtype=float)
    orientations = np.zeros((21, 4), dtype=float)
    orientations[:, 0] = 1.0
    for joint in range(2, 21):
        parent = PARENTS[joint]
        positions[joint] = positions[parent] + rotate(orientations[parent], offsets[joint])
        orientations[joint] = multiply(
            orientations[parent], euler_quaternion(rotations[joint], order)
        )
    return positions[list(TIP_INDICES)].reshape(-1)


def collect(sock: socket.socket, duration: float, order: str, hand: str) -> np.ndarray:
    rows = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            payload = json.loads(sock.recvfrom(65536)[0])
            rows.append(tip_feature(payload, order, hand))
        except (KeyError, ValueError, json.JSONDecodeError, socket.timeout):
            continue
    if len(rows) < 30:
        raise RuntimeError("有效帧不足，请确认 Unity/Hi5 正在发送 21 组 Position 和 Rotation")
    return np.asarray(rows)


def main() -> int:
    default_config = Path(__file__).resolve().parents[1] / "build" / "bin" / "thumb_glove_calibration.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--ip", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=25001)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    config_path = args.config.resolve()
    payload = json.loads(config_path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 2 or payload.get("model") != "single_thumb_fixed_wrist":
        raise RuntimeError("锚点标定仅保留在 schema 2 single_thumb_fixed_wrist 方案中")
    order = payload.get("euler_order", "zyx")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.settimeout(0.25)
    samples = {}
    try:
        print("=== schema 2：两接触姿态锚点标定 ===")
        for key, instruction in STAGES:
            input(f"\n请摆好：{instruction}\n准备好后按回车，采集 {args.duration:g} 秒...")
            samples[key] = collect(sock, args.duration, order, args.hand)
            print(f"已采集 {len(samples[key])} 帧")
    finally:
        sock.close()

    centers = {key: np.median(value, axis=0) for key, value in samples.items()}
    separation = float(np.linalg.norm(centers[STAGES[0][0]] - centers[STAGES[1][0]]))
    if separation < 0.5:
        raise RuntimeError(f"两个接触姿态过于接近（距离 {separation:.3f}），请重新标定")

    targets = {
        "thumb_index_touch": {"1": 750, "2": 300, "3": 350},
        "thumb_middle_touch": {"1": 750, "2": 0, "4": 350},
    }
    anchors = {}
    for key, _instruction in STAGES:
        deviations = np.linalg.norm(samples[key] - centers[key], axis=1)
        noise95 = float(np.percentile(deviations, 95))
        radius = min(max(noise95 * 3.0, separation * 0.20, 0.25), separation * 0.45)
        anchors[key] = {
            "enabled": True,
            "tip_positions_hand_local": centers[key].tolist(),
            "activation_radius": radius,
            "targets": targets[key],
            "sample_count": int(len(samples[key])),
            "p95_noise_radius": noise95,
        }

    payload["gesture_anchors"] = anchors
    payload["gesture_anchor_model"] = {
        "method": "hand_local_forward_kinematics_tip_positions",
        "tip_joint_indices": list(TIP_INDICES),
        "wrist_rotation_included": False,
        "euler_order": order,
        "anchor_separation": separation,
        "calibrated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    distance_features = {}
    for key, center in centers.items():
        tips = center.reshape(3, 3)
        distance_features[key] = np.array([
            np.linalg.norm(tips[0] - tips[1]),
            np.linalg.norm(tips[0] - tips[2]),
        ])
    distance_separation = float(np.linalg.norm(
        distance_features[STAGES[0][0]] - distance_features[STAGES[1][0]]
    ))
    payload["thumb_position_fusion"] = {
        "enabled": True,
        "method": "thumb_index_middle_distance_segment",
        "support_radius": max(1.0, distance_separation * 0.8),
        "influence": 0.8,
        "distance_feature_separation": distance_separation,
    }
    temporary = config_path.with_suffix(config_path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(config_path)
    print(f"\n锚点标定完成，已写入：{config_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
