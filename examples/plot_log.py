import csv
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt



FORCE_THRESHOLD = 500
SOFT_CURRENT_WARN_MA = 550
SOFT_CURRENT_RELEASE_MA = 680
HARD_CURRENT_LIMIT_MA = 1000
JOINT_IDS = [f"ID{i}" for i in range(1, 7)]
INVALID_POSITION_THRESHOLD = 10000
DISPLAY_SOURCE_MAP = {
    # Display-layer remap only for command-side curves.
}
REAL_SOURCE_MAP = {
    # Commands and feedback now both use physical hardware order:
    # ID1 thumb pitch, ID2 thumb yaw.
}
DEFAULT_LOG_CANDIDATES = [
    Path("build/bin/admittance_log.csv"),
    Path("admittance_log.csv"),
]


def _normalize_joint_id(raw_id):
    text = str(raw_id).strip().upper()
    if text.startswith("ID"):
        suffix = text[2:]
        if suffix.isdigit():
            return f"ID{int(suffix)}"
        return text
    if text.isdigit():
        return f"ID{int(text)}"
    return text


def _parse_row(row):
    if not row:
        return None

    try:
        if len(row) >= 9:
            t_ms = int(row[0])
            joint_id = _normalize_joint_id(row[2])
            raw_desired = int(row[3])
            shaped_desired = int(row[4])
            cmd = int(row[5])
            real = int(row[6])
            amp = int(row[7])
            offset = int(row[8])
            return t_ms, joint_id, raw_desired, shaped_desired, cmd, real, amp, offset

        if row[0].strip() == "DATA" and len(row) >= 7:
            t_ms = int(row[1])
            joint_id = _normalize_joint_id(row[2])
            raw_desired = int(row[3])
            shaped_desired = raw_desired
            cmd = int(row[4])
            real = int(row[5])
            amp = int(row[6])
            offset = cmd - shaped_desired
            return t_ms, joint_id, raw_desired, shaped_desired, cmd, real, amp, offset

        if len(row) >= 8:
            t_ms = int(row[0])
            joint_id = _normalize_joint_id(row[1])
            raw_desired = int(row[2])
            shaped_desired = int(row[3])
            cmd = int(row[4])
            real = int(row[5])
            amp = int(row[6])
            offset = int(row[7])
            return t_ms, joint_id, raw_desired, shaped_desired, cmd, real, amp, offset

        if len(row) >= 6:
            t_ms = int(row[0])
            joint_id = _normalize_joint_id(row[1])
            raw_desired = int(row[2])
            shaped_desired = raw_desired
            cmd = int(row[3])
            real = int(row[4])
            amp = int(row[5])
            offset = cmd - shaped_desired
            return t_ms, joint_id, raw_desired, shaped_desired, cmd, real, amp, offset
    except ValueError:
        return None

    return None


def _empty_joint_data():
    return {
        "times": [],
        "raw_desired": [],
        "shaped_desired": [],
        "cmd": [],
        "real": [],
        "amp": [],
        "offset": [],
    }


def _sanitize_real_position(real_val):
    if 0 <= real_val <= INVALID_POSITION_THRESHOLD:
        return real_val
    return None


def _resolve_log_file(argv_path=None):
    if argv_path:
        return Path(argv_path)

    for candidate in DEFAULT_LOG_CANDIDATES:
        if candidate.exists():
            return candidate

    return DEFAULT_LOG_CANDIDATES[0]


def _print_summary(joint_data, valid_joint_ids):
    print("\nJoint summary:")
    for joint_id in valid_joint_ids:
        source_joint_id = DISPLAY_SOURCE_MAP.get(joint_id, joint_id)
        real_source_joint_id = REAL_SOURCE_MAP.get(joint_id, joint_id)
        data = joint_data[source_joint_id]
        real_data = joint_data[real_source_joint_id]

        amp_values = data["amp"]
        offset_values = data["offset"]
        real_values = [v for v in real_data["real"] if v is not None]
        warn_hits = sum(1 for v in amp_values if v >= SOFT_CURRENT_WARN_MA)
        release_hits = sum(1 for v in amp_values if v >= SOFT_CURRENT_RELEASE_MA)
        hard_hits = sum(1 for v in amp_values if v >= HARD_CURRENT_LIMIT_MA)

        print(
            f"  {joint_id}: samples={len(data['times'])}, amp_max={max(amp_values) if amp_values else 0}, "
            f"offset_max={max(offset_values) if offset_values else 0}, real_max={max(real_values) if real_values else 'n/a'}, "
            f"warn_hits={warn_hits}, release_hits={release_hits}, hard_hits={hard_hits}"
        )


def plot_log(filename):
    joint_data = {joint_id: _empty_joint_data() for joint_id in JOINT_IDS}
    start_time = None

    print(f"Reading file: {filename}")

    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        next(reader, None)

        for row in reader:
            parsed = _parse_row(row)
            if parsed is None:
                continue

            t_ms, joint_id, raw_val, shaped_val, cmd_val, real_val, amp_val, offset_val = parsed
            if joint_id not in joint_data:
                joint_data[joint_id] = _empty_joint_data()

            if start_time is None:
                start_time = t_ms

            series = joint_data[joint_id]
            series["times"].append((t_ms - start_time) / 1000.0)
            series["raw_desired"].append(raw_val)
            series["shaped_desired"].append(shaped_val)
            series["cmd"].append(cmd_val)
            series["real"].append(_sanitize_real_position(real_val))
            series["amp"].append(amp_val)
            series["offset"].append(offset_val)

    valid_joint_ids = [joint_id for joint_id, data in joint_data.items() if data["times"]]
    if not valid_joint_ids:
        print("No valid data found.")
        return

    _print_summary(joint_data, valid_joint_ids)

    fig, axes = plt.subplots(len(valid_joint_ids), 3, figsize=(18, 3.5 * len(valid_joint_ids)), sharex="col")
    if len(valid_joint_ids) == 1:
        axes = [axes]

    fig.suptitle(f"Admittance Effect Check ({Path(filename).name})", fontsize=14)

    for row_idx, joint_id in enumerate(valid_joint_ids):
        source_joint_id = DISPLAY_SOURCE_MAP.get(joint_id, joint_id)
        real_source_joint_id = REAL_SOURCE_MAP.get(joint_id, joint_id)
        data = joint_data.get(source_joint_id, _empty_joint_data())
        real_data = joint_data.get(real_source_joint_id, _empty_joint_data())
        ax_pos, ax_offset, ax_current = axes[row_idx]

        ax_pos.plot(data["times"], data["raw_desired"], label="Raw Desired", color="steelblue", linestyle=":")
        ax_pos.plot(data["times"], data["shaped_desired"], label="Shaped Desired", color="royalblue", linestyle="--")
        ax_pos.plot(data["times"], data["cmd"], label="Admittance Cmd", color="seagreen", linewidth=2)
        ax_pos.plot(real_data["times"], real_data["real"], label="Real Position", color="firebrick", alpha=0.8)
        ax_pos.set_ylabel(f"{joint_id}\nPosition")
        ax_pos.grid(True)
        if row_idx == 0:
            ax_pos.set_title("Position")
            ax_pos.legend(fontsize=8)

        ax_offset.plot(data["times"], data["offset"], label="Admittance Offset", color="purple", linewidth=2)
        ax_offset.axhline(y=0, color="black", linestyle=":")
        ax_offset.set_ylabel("Offset")
        ax_offset.grid(True)
        if row_idx == 0:
            ax_offset.set_title("Offset")
            ax_offset.legend(fontsize=8)

        ax_current.plot(data["times"], data["amp"], label="Filtered Current (mA)", color="darkorange")
        ax_current.axhline(y=FORCE_THRESHOLD, color="goldenrod", linestyle="--", label=f"Force Threshold ({FORCE_THRESHOLD})")
        ax_current.axhline(y=SOFT_CURRENT_WARN_MA, color="tomato", linestyle="--", label=f"Soft Warn ({SOFT_CURRENT_WARN_MA})")
        ax_current.axhline(y=SOFT_CURRENT_RELEASE_MA, color="crimson", linestyle="-.", label=f"Soft Release ({SOFT_CURRENT_RELEASE_MA})")
        ax_current.axhline(y=HARD_CURRENT_LIMIT_MA, color="black", linestyle=":", label=f"Hardware Limit ({HARD_CURRENT_LIMIT_MA})")
        ax_current.set_ylabel("Current (mA)")
        ax_current.grid(True)
        if row_idx == 0:
            ax_current.set_title("Current")
            ax_current.legend(fontsize=8)

    for col in range(3):
        axes[-1][col].set_xlabel("Time (s)")

    plt.tight_layout(rect=(0, 0, 1, 0.98))
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        log_file = _resolve_log_file(sys.argv[1])
    else:
        log_file = _resolve_log_file()

    if not log_file.exists():
        print(f"File not found: {log_file}")
        print("Usage: python plot_log.py <logfile.csv>")
        sys.exit(1)

    plot_log(str(log_file))
