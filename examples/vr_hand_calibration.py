#!/usr/bin/env python3
"""Collect Unity XR Hands poses and recommend VrHandCalibration values.

Default use:
  python examples/vr_hand_calibration.py --hand right --apply-cpp

Run this while the PICO Unity app is sending RightHandUnityUdpTcpSender packets
through adb reverse tcp:5006 tcp:5006. Stop unity_hand_udp_bridge.py and
udp_receiver_unity.exe during calibration so this script can own the port.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import socket
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


TCP_HOST = "127.0.0.1"
TCP_PORT = 5006
UDP_HOST = "127.0.0.1"
UDP_PORT = 25001
BUFFER_SIZE = 65536
STATE_DURATION_SEC = 4.0

STATES = [
    ("natural_open", "自然张开"),
    ("four_finger_fist", "四指握拳"),
    ("thumb_bend_no_opposition", "拇指弯曲（不对掌）"),
    ("thumb_opposition_no_bend", "拇指对掌（不弯曲）"),
    ("thumb_fist_opposition", "拇指握拳对掌"),
]

STATE_HELP = {
    "natural_open": "五指自然张开，手掌放松。",
    "four_finger_fist": "食指/中指/无名指/小指握拳，拇指尽量保持自然不参与对掌。",
    "thumb_bend_no_opposition": "只弯曲拇指，尽量不要横向对掌。",
    "thumb_opposition_no_bend": "拇指横向对掌，但拇指本身尽量伸直。",
    "thumb_fist_opposition": "拇指弯曲并对掌，类似握拳时拇指压向掌心。",
}


def get_joint(values: List[float], idx: int) -> Tuple[float, float, float]:
    base = idx * 3
    if base + 2 >= len(values):
        raise IndexError(f"joint index {idx} out of range")
    return values[base], values[base + 1], values[base + 2]


def sub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(dot(v, v))


def normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    n = norm(v)
    if n < 1e-9:
        return 0.0, 0.0, 0.0
    return v[0] / n, v[1] / n, v[2] / n


def clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def angle_deg_between(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    denom = norm(a) * norm(b)
    if denom < 1e-9:
        return 0.0
    return math.degrees(math.acos(clip(dot(a, b) / denom, -1.0, 1.0)))


def joint_angle_deg(
    parent: Tuple[float, float, float],
    joint: Tuple[float, float, float],
    child: Tuple[float, float, float],
) -> float:
    return angle_deg_between(sub(parent, joint), sub(child, joint))


def source_key(hand: str) -> str:
    return f"{hand}Positions"


def extract_vr_features(packet: Dict[str, Any], hand: str) -> Dict[str, float]:
    pos_key = source_key(hand)
    pos = packet.get(pos_key)
    if not isinstance(pos, list) or len(pos) < 63:
        raise ValueError(f"missing or short {pos_key}")
    if not all(isinstance(v, (int, float)) for v in pos):
        raise ValueError(f"{pos_key} contains non-number values")

    wrist = get_joint(pos, 0)
    thumb_cmc = get_joint(pos, 1)
    thumb_mcp = get_joint(pos, 2)
    thumb_tip = get_joint(pos, 4)
    index_mcp = get_joint(pos, 5)
    index_pip = get_joint(pos, 6)
    index_tip = get_joint(pos, 8)
    middle_mcp = get_joint(pos, 9)
    middle_pip = get_joint(pos, 10)
    middle_tip = get_joint(pos, 12)
    ring_mcp = get_joint(pos, 13)
    ring_pip = get_joint(pos, 14)
    ring_tip = get_joint(pos, 16)
    pinky_mcp = get_joint(pos, 17)
    pinky_pip = get_joint(pos, 18)
    pinky_tip = get_joint(pos, 20)

    palm_x = normalize(sub(index_mcp, wrist))
    palm_y = normalize(sub(middle_mcp, wrist))
    palm_z = normalize(cross(palm_x, palm_y))
    thumb_vec = normalize(sub(thumb_tip, thumb_cmc))

    return {
        "frame_id": float(packet.get("frameId", -1)),
        "thumb_yaw": abs(dot(thumb_vec, palm_x)),
        "thumb_pitch": joint_angle_deg(thumb_cmc, thumb_mcp, thumb_tip),
        "index": joint_angle_deg(index_mcp, index_pip, index_tip),
        "middle": joint_angle_deg(middle_mcp, middle_pip, middle_tip),
        "ring": joint_angle_deg(ring_mcp, ring_pip, ring_tip),
        "pinky": joint_angle_deg(pinky_mcp, pinky_pip, pinky_tip),
        "palm_normal_x": palm_z[0],
        "palm_normal_y": palm_z[1],
        "palm_normal_z": palm_z[2],
    }


def summarize_series(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "stdev": None, "min": None, "max": None}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "stdev": 0.0 if len(values) == 1 else statistics.pstdev(values),
        "min": min(values),
        "max": max(values),
    }


def summarize_samples(samples: List[Dict[str, float]]) -> Dict[str, Dict[str, Optional[float]]]:
    if not samples:
        return {}
    keys = [k for k, v in samples[0].items() if isinstance(v, (int, float)) and k != "frame_id"]
    return {key: summarize_series([float(s[key]) for s in samples if key in s]) for key in keys}


def collect_feature_values(samples_by_state: Dict[str, List[Dict[str, float]]], feature: str, states: Iterable[str]) -> List[float]:
    values: List[float] = []
    for state in states:
        values.extend(float(sample[feature]) for sample in samples_by_state.get(state, []) if feature in sample)
    return values


def robust_median(values: List[float]) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) >= 10:
        lo = int(len(ordered) * 0.10)
        hi = max(lo + 1, int(len(ordered) * 0.90))
        ordered = ordered[lo:hi]
    return statistics.median(ordered)


def choose_range(open_values: List[float], close_values: List[float], fallback_open: float, fallback_close: float) -> Tuple[float, float]:
    open_value = robust_median(open_values)
    close_value = robust_median(close_values)
    if open_value is None:
        open_value = fallback_open
    if close_value is None:
        close_value = fallback_close
    if abs(open_value - close_value) < 1e-6:
        close_value = fallback_close
    return float(open_value), float(close_value)


def build_recommendation(samples_by_state: Dict[str, List[Dict[str, float]]]) -> Dict[str, Any]:
    thumb_yaw_open, thumb_yaw_close = choose_range(
        collect_feature_values(samples_by_state, "thumb_yaw", ["natural_open", "thumb_bend_no_opposition"]),
        collect_feature_values(samples_by_state, "thumb_yaw", ["thumb_opposition_no_bend", "thumb_fist_opposition"]),
        0.20,
        0.78,
    )
    thumb_pitch_open, thumb_pitch_close = choose_range(
        collect_feature_values(samples_by_state, "thumb_pitch", ["natural_open", "thumb_opposition_no_bend"]),
        collect_feature_values(samples_by_state, "thumb_pitch", ["thumb_bend_no_opposition", "thumb_fist_opposition"]),
        175.0,
        88.0,
    )

    recommendation = {
        "thumb_yaw_open": thumb_yaw_open,
        "thumb_yaw_close": thumb_yaw_close,
        "thumb_pitch_open": thumb_pitch_open,
        "thumb_pitch_close": thumb_pitch_close,
        "index_open": choose_range(
            collect_feature_values(samples_by_state, "index", ["natural_open"]),
            collect_feature_values(samples_by_state, "index", ["four_finger_fist"]),
            176.0,
            78.0,
        )[0],
        "index_close": choose_range(
            collect_feature_values(samples_by_state, "index", ["natural_open"]),
            collect_feature_values(samples_by_state, "index", ["four_finger_fist"]),
            176.0,
            78.0,
        )[1],
        "middle_open": choose_range(
            collect_feature_values(samples_by_state, "middle", ["natural_open"]),
            collect_feature_values(samples_by_state, "middle", ["four_finger_fist"]),
            176.0,
            76.0,
        )[0],
        "middle_close": choose_range(
            collect_feature_values(samples_by_state, "middle", ["natural_open"]),
            collect_feature_values(samples_by_state, "middle", ["four_finger_fist"]),
            176.0,
            76.0,
        )[1],
        "ring_open": choose_range(
            collect_feature_values(samples_by_state, "ring", ["natural_open"]),
            collect_feature_values(samples_by_state, "ring", ["four_finger_fist"]),
            176.0,
            76.0,
        )[0],
        "ring_close": choose_range(
            collect_feature_values(samples_by_state, "ring", ["natural_open"]),
            collect_feature_values(samples_by_state, "ring", ["four_finger_fist"]),
            176.0,
            76.0,
        )[1],
        "pinky_open": choose_range(
            collect_feature_values(samples_by_state, "pinky", ["natural_open"]),
            collect_feature_values(samples_by_state, "pinky", ["four_finger_fist"]),
            176.0,
            74.0,
        )[0],
        "pinky_close": choose_range(
            collect_feature_values(samples_by_state, "pinky", ["natural_open"]),
            collect_feature_values(samples_by_state, "pinky", ["four_finger_fist"]),
            176.0,
            74.0,
        )[1],
        "open_deadzone": 0.05,
        "close_deadzone": 0.08,
        "thumb_yaw_sensitivity": 1.0,
        "thumb_pitch_sensitivity": 1.0,
    }
    return recommendation


def calibration_block(values: Dict[str, Any]) -> str:
    order = [
        "thumb_yaw_open",
        "thumb_yaw_close",
        "thumb_pitch_open",
        "thumb_pitch_close",
        "index_open",
        "index_close",
        "middle_open",
        "middle_close",
        "ring_open",
        "ring_close",
        "pinky_open",
        "pinky_close",
        "open_deadzone",
        "close_deadzone",
        "thumb_yaw_sensitivity",
        "thumb_pitch_sensitivity",
    ]
    lines = ["struct VrHandCalibration {"]
    for key in order:
        lines.append(f"    double {key} = {float(values[key]):.6g};")
    lines.append("};")
    return "\n".join(lines)


def apply_cpp(cpp_path: Path, values: Dict[str, Any]) -> None:
    text = cpp_path.read_text(encoding="utf-8")
    new_block = calibration_block(values)
    pattern = r"struct VrHandCalibration \{.*?\n\};"
    updated, count = re.subn(pattern, new_block, text, count=1, flags=re.S)
    if count != 1:
        raise RuntimeError(f"failed to locate VrHandCalibration in {cpp_path}")
    cpp_path.write_text(updated, encoding="utf-8", newline="")


class PacketReader:
    def receive(self) -> Dict[str, Any]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class TcpPacketReader(PacketReader):
    def __init__(self, host: str, port: int):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(1)
        print(f"等待 Unity TCP 连接: {host}:{port}")
        self.client, addr = self.server.accept()
        self.client.settimeout(1.0)
        self.buffer = b""
        print(f"Unity 已连接: {addr[0]}:{addr[1]}")

    def receive(self) -> Dict[str, Any]:
        while True:
            while b"\n" not in self.buffer:
                chunk = self.client.recv(BUFFER_SIZE)
                if not chunk:
                    raise RuntimeError("Unity TCP disconnected")
                self.buffer += chunk
            raw, self.buffer = self.buffer.split(b"\n", 1)
            try:
                return json.loads(raw.decode("utf-8"))
            except Exception:
                continue

    def close(self) -> None:
        try:
            self.client.close()
        except Exception:
            pass
        self.server.close()


class UdpPacketReader(PacketReader):
    def __init__(self, host: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(1.0)
        print(f"监听 UDP: {host}:{port}")

    def receive(self) -> Dict[str, Any]:
        while True:
            raw, _ = self.sock.recvfrom(BUFFER_SIZE)
            try:
                return json.loads(raw.decode("utf-8"))
            except Exception:
                continue

    def close(self) -> None:
        self.sock.close()


def collect_state(reader: PacketReader, hand: str, state_key: str, label: str, duration_sec: float) -> List[Dict[str, float]]:
    print("")
    print(f"[{label}] {STATE_HELP[state_key]}")
    input(f"摆好姿态后按回车，采集 {duration_sec:.1f} 秒...")

    samples: List[Dict[str, float]] = []
    deadline = time.time() + duration_sec
    while time.time() < deadline:
        try:
            packet = reader.receive()
            feature = extract_vr_features(packet, hand)
            feature["state"] = state_key  # type: ignore[assignment]
            samples.append(feature)
        except socket.timeout:
            continue
        except Exception:
            continue
    print(f"完成: {label}, 有效样本 {len(samples)}")
    return samples


def print_summary(summary_by_state: Dict[str, Any], recommendation: Dict[str, Any]) -> None:
    print("\n=== 姿态特征中位数 ===")
    for state_key, label in STATES:
        summary = summary_by_state.get(state_key, {})
        parts = []
        for key in ["thumb_yaw", "thumb_pitch", "index", "middle", "ring", "pinky"]:
            value = summary.get(key, {}).get("median")
            parts.append(f"{key}={'None' if value is None else f'{value:.3f}'}")
        print(f"{label}: " + ", ".join(parts))

    print("\n=== 推荐写入 VrHandCalibration ===")
    print(calibration_block(recommendation))


def main() -> None:
    default_cpp = Path(__file__).with_name("udp_receiver_unity.cpp")
    parser = argparse.ArgumentParser(description="Calibrate independent VR-hand-tracking mapping for WA100.")
    parser.add_argument("--hand", choices=["left", "right"], default="right")
    parser.add_argument("--source", choices=["tcp", "udp"], default="tcp")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--duration", type=float, default=STATE_DURATION_SEC)
    parser.add_argument("--output", default="vr_hand_calibration_result.json")
    parser.add_argument("--apply-cpp", action="store_true", help="rewrite VrHandCalibration in udp_receiver_unity.cpp")
    parser.add_argument("--cpp", default=str(default_cpp), help="path to udp_receiver_unity.cpp")
    parser.add_argument("--from-result", default=None, help="apply/reprint an existing result JSON without collecting")
    args = parser.parse_args()

    if args.from_result:
        payload = json.loads(Path(args.from_result).read_text(encoding="utf-8-sig"))
        recommendation = payload["recommendation"]
        print_summary(payload.get("summary_by_state", {}), recommendation)
        if args.apply_cpp:
            apply_cpp(Path(args.cpp), recommendation)
            print(f"\n已更新: {args.cpp}")
        return

    host = args.host or (TCP_HOST if args.source == "tcp" else UDP_HOST)
    port = args.port or (TCP_PORT if args.source == "tcp" else UDP_PORT)
    reader: PacketReader
    reader = TcpPacketReader(host, port) if args.source == "tcp" else UdpPacketReader(host, port)

    samples_by_state: Dict[str, List[Dict[str, float]]] = {}
    try:
        print("采集姿态:")
        for idx, (_, label) in enumerate(STATES, start=1):
            print(f"  {idx}. {label}")
        for state_key, label in STATES:
            samples_by_state[state_key] = collect_state(reader, args.hand, state_key, label, args.duration)
    finally:
        reader.close()

    summary_by_state = {state: summarize_samples(samples) for state, samples in samples_by_state.items()}
    recommendation = build_recommendation(samples_by_state)
    print_summary(summary_by_state, recommendation)

    payload = {
        "hand": args.hand,
        "source": args.source,
        "duration_sec_per_state": args.duration,
        "states": [state for state, _ in STATES],
        "summary_by_state": summary_by_state,
        "recommendation": recommendation,
        "sample_count": sum(len(samples) for samples in samples_by_state.values()),
    }
    out_path = Path(args.output)
    out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n结果已保存: {out_path}")

    if args.apply_cpp:
        apply_cpp(Path(args.cpp), recommendation)
        print(f"已更新: {args.cpp}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已中断")
        sys.exit(1)
