import argparse
import json
import math
import socket
import statistics
import sys
import time
from pathlib import Path


UDP_IP = "127.0.0.1"
UDP_PORT = 25001
BUFFER_SIZE = 65536
STATE_DURATION_SEC = 5.0


STATES = [
    ("natural_open", "自然张开"),
    ("straight_perp_palm", "伸直垂直掌心"),
    ("natural_bent", "自然弯曲"),
    ("fist_bent", "握拳弯曲"),
]


def get_joint(values, idx):
    base = idx * 3
    if base + 2 >= len(values):
        raise IndexError(f"joint index {idx} out of range")
    return values[base], values[base + 1], values[base + 2]


def wrap_angle_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def angle_diff_deg(a, b):
    return wrap_angle_deg(a - b)


def extract_thumb_features(packet, hand):
    rot_key = f"{hand}Rotations"
    if rot_key not in packet:
        raise KeyError(f"missing key: {rot_key}")

    rot = packet[rot_key]
    if len(rot) < 63:
        raise ValueError("hand rotation array too short")

    thumb_cmc_r = get_joint(rot, 1)
    thumb_mcp_r = get_joint(rot, 2)
    thumb_ip_r = get_joint(rot, 3)

    mcp_rel_x = angle_diff_deg(thumb_mcp_r[0], thumb_cmc_r[0])
    mcp_rel_y = angle_diff_deg(thumb_mcp_r[1], thumb_cmc_r[1])
    mcp_rel_z = angle_diff_deg(thumb_mcp_r[2], thumb_cmc_r[2])
    ip_rel_x = angle_diff_deg(thumb_ip_r[0], thumb_mcp_r[0])
    ip_rel_y = angle_diff_deg(thumb_ip_r[1], thumb_mcp_r[1])
    ip_rel_z = angle_diff_deg(thumb_ip_r[2], thumb_mcp_r[2])

    return {
        "frame_id": packet.get("frameId"),
        "mcp_rel_x": mcp_rel_x,
        "mcp_rel_y": mcp_rel_y,
        "mcp_rel_z": mcp_rel_z,
        "ip_rel_x": ip_rel_x,
        "ip_rel_y": ip_rel_y,
        "ip_rel_z": ip_rel_z,
        "thumb_yaw_signed": ip_rel_z,
        "thumb_pitch": abs(ip_rel_y),
    }


def summarize_series(values):
    if not values:
        return {"count": 0, "mean": None, "stdev": None, "min": None, "max": None}
    stdev = 0.0 if len(values) == 1 else statistics.pstdev(values)
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "stdev": stdev,
        "min": min(values),
        "max": max(values),
    }


def summarize_samples(samples):
    if not samples:
        return {}
    keys = []
    for k, v in samples[0].items():
        if k == "frame_id":
            continue
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            keys.append(k)
    return {key: summarize_series([s[key] for s in samples]) for key in keys}


def receive_packet(sock):
    while True:
        data, _ = sock.recvfrom(BUFFER_SIZE)
        try:
            return json.loads(data.decode("utf-8"))
        except Exception:
            continue


def collect_state(sock, hand, state_key, state_label, duration_sec):
    input(f"\n准备开始 [{state_label}]，按回车采集 {duration_sec:.0f} 秒...")
    print(f"采集中: {state_label}")

    samples = []
    deadline = time.time() + duration_sec
    next_log_sec = math.floor(duration_sec)

    while True:
        now = time.time()
        if now >= deadline:
            break

        remaining = deadline - now
        sock.settimeout(max(0.1, min(1.0, remaining)))
        try:
            packet = receive_packet(sock)
            feature = extract_thumb_features(packet, hand)
            feature["state"] = state_key
            samples.append(feature)
        except socket.timeout:
            continue
        except Exception:
            continue

        whole_remaining = math.ceil(deadline - time.time())
        if whole_remaining < next_log_sec:
            next_log_sec = whole_remaining
            print(f"  剩余 {max(whole_remaining, 0)}s, 已采样 {len(samples)} 帧")

    print(f"完成: {state_label}, 共采样 {len(samples)} 帧")
    return samples


def collect_feature_values(samples_by_state, feature_name, state_keys):
    values = []
    for state_key in state_keys:
        for sample in samples_by_state.get(state_key, []):
            if feature_name in sample:
                values.append(sample[feature_name])
    return values


def percentile(sorted_values, q):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return sorted_values[lo]
    frac = pos - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def robust_range(values, q_low=0.05, q_high=0.95):
    if not values:
        return None, None
    sorted_values = sorted(values)
    return percentile(sorted_values, q_low), percentile(sorted_values, q_high)


def build_recommendation(samples_by_state, summary_by_state):
    yaw_values = collect_feature_values(
        samples_by_state,
        "thumb_yaw_signed",
        ["natural_open", "straight_perp_palm", "natural_bent", "fist_bent"],
    )
    pitch_open_values = collect_feature_values(
        samples_by_state,
        "thumb_pitch",
        ["natural_open", "straight_perp_palm"],
    )
    pitch_close_values = collect_feature_values(
        samples_by_state,
        "thumb_pitch",
        ["natural_bent", "fist_bent"],
    )

    yaw_min, yaw_max = robust_range(yaw_values)
    pitch_open, _ = robust_range(pitch_open_values, 0.05, 0.90)
    _, pitch_close = robust_range(pitch_close_values, 0.10, 0.95)

    warnings = []
    if yaw_min is None or yaw_max is None:
        warnings.append("yaw range unavailable")
    if pitch_open is None or pitch_close is None:
        warnings.append("pitch range unavailable")

    if yaw_min is not None and yaw_max is not None and yaw_max <= yaw_min:
        yaw_min = min(yaw_values)
        yaw_max = max(yaw_values)
        warnings.append("yaw range overlapped; fell back to raw min/max")

    if pitch_open is not None and pitch_close is not None and pitch_close <= pitch_open:
        pitch_open = min(pitch_open_values)
        pitch_close = max(pitch_close_values)
        warnings.append("pitch range overlapped; fell back to raw min/max")

    return {
        "thumb_yaw_feature": "thumb_yaw_signed",
        "thumb_pitch_feature": "thumb_pitch",
        "thumb_yaw_min": yaw_min,
        "thumb_yaw_max": yaw_max,
        "thumb_pitch_min": pitch_open,
        "thumb_pitch_max": pitch_close,
        "yaw_state_means": {
            state: summary_by_state.get(state, {}).get("thumb_yaw_signed", {}).get("mean")
            for state, _ in STATES
        },
        "pitch_state_means": {
            state: summary_by_state.get(state, {}).get("thumb_pitch", {}).get("mean")
            for state, _ in STATES
        },
        "warnings": warnings,
    }


def print_recommendation(rec):
    print("\n=== 当前映射对应的范围建议 ===")
    print(f"yaw 特征: {rec['thumb_yaw_feature']}")
    print(f"pitch 特征: {rec['thumb_pitch_feature']}")
    if rec["thumb_yaw_min"] is not None and rec["thumb_yaw_max"] is not None:
        print(f"const double THUMB_YAW_MIN = {rec['thumb_yaw_min']:.2f};")
        print(f"const double THUMB_YAW_MAX = {rec['thumb_yaw_max']:.2f};")
    if rec["thumb_pitch_min"] is not None and rec["thumb_pitch_max"] is not None:
        print(f"const double THUMB_PITCH_MIN = {rec['thumb_pitch_min']:.2f};")
        print(f"const double THUMB_PITCH_MAX = {rec['thumb_pitch_max']:.2f};")

    print("")
    print("各姿态均值:")
    for state_key, state_label in STATES:
        yaw_mean = rec["yaw_state_means"].get(state_key)
        pitch_mean = rec["pitch_state_means"].get(state_key)
        yaw_text = "None" if yaw_mean is None else f"{yaw_mean:.2f}"
        pitch_text = "None" if pitch_mean is None else f"{pitch_mean:.2f}"
        print(f"  {state_label}: yaw={yaw_text}, pitch={pitch_text}")

    if rec.get("warnings"):
        print("")
        for warning in rec["warnings"]:
            print(f"警告: {warning}")


def main():
    parser = argparse.ArgumentParser(description="Unity thumb range calibration collector")
    parser.add_argument("--hand", choices=["left", "right"], default="left")
    parser.add_argument("--ip", default=UDP_IP)
    parser.add_argument("--port", type=int, default=UDP_PORT)
    parser.add_argument("--duration", type=float, default=STATE_DURATION_SEC)
    parser.add_argument("--output", default="thumb_calibration_result.json")
    args = parser.parse_args()

    out_path = Path(args.output)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))

    print(f"监听 UDP: {args.ip}:{args.port}")
    print(f"手: {args.hand}")
    print("本脚本按当前 udp_receiver_unity.cpp 的逻辑，只标定范围。")
    print("使用的四个姿态:")
    for idx, (_, label) in enumerate(STATES, start=1):
        print(f"  {idx}. {label}")
    print("")
    print("说明:")
    print("  1. yaw 使用当前 C++ 的 thumb_ip_rel_z。")
    print("  2. pitch 使用当前 C++ 的 abs(thumb_ip_rel_y)。")
    print("  3. 重点输出 THUMB_YAW_MIN/MAX 和 THUMB_PITCH_MIN/MAX。")

    all_samples = []
    samples_by_state = {}
    summary_by_state = {}

    try:
        for state_key, state_label in STATES:
            samples = collect_state(sock, args.hand, state_key, state_label, args.duration)
            if not samples:
                print(f"警告: [{state_label}] 没采到有效数据")
            all_samples.extend(samples)
            samples_by_state[state_key] = samples
            summary_by_state[state_key] = summarize_samples(samples)

        recommendation = build_recommendation(samples_by_state, summary_by_state)
        print_recommendation(recommendation)

        payload = {
            "hand": args.hand,
            "duration_sec_per_state": args.duration,
            "states": [state_key for state_key, _ in STATES],
            "summary_by_state": summary_by_state,
            "recommendation": recommendation,
            "sample_count": len(all_samples),
        }
        out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n结果已保存到: {out_path}")
    finally:
        sock.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已中断")
        sys.exit(1)
