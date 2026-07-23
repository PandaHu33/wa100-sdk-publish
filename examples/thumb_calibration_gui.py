#!/usr/bin/env python3
"""Tk GUI for guided online Hi5 thumb calibration."""

from __future__ import annotations

import argparse
import json
import queue
import socket
import threading
import time
from pathlib import Path
from tkinter import BOTH, DISABLED, END, LEFT, NORMAL, RIGHT, StringVar, Tk, messagebox
from tkinter import ttk

import numpy as np

from thumb_calibration import (
    BUFFER_SIZE,
    EULER_ORDERS,
    STATES,
    UDP_IP,
    UDP_PORT,
    calibration_payload,
    fit_candidate,
    receive_packet,
    save_samples,
)


class CalibrationWindow:
    def __init__(self, root: Tk, hand: str, host: str, port: int, output: Path, diagnostics: Path, samples_path: Path, duration: float | None):
        self.root = root
        self.hand = hand
        self.output = output
        self.diagnostics = diagnostics
        self.samples_path = samples_path
        self.duration_override = duration
        self.samples: dict[str, list[np.ndarray]] = {}
        self.state_index = 0
        self.worker: threading.Thread | None = None
        self.events: queue.Queue = queue.Queue()
        self.deadline = 0.0
        self.closed = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.25)
        self.sock.bind((host, port))

        root.title("Hi5 右手套拇指在线标定")
        root.geometry("760x570")
        root.minsize(680, 520)
        root.attributes("-topmost", True)
        root.after(1500, lambda: root.attributes("-topmost", False))
        root.protocol("WM_DELETE_WINDOW", self.close)

        outer = ttk.Frame(root, padding=18)
        outer.pack(fill=BOTH, expand=True)
        self.stage_var = StringVar()
        self.title_var = StringVar()
        self.instruction_var = StringVar()
        self.countdown_var = StringVar(value="准备开始")
        ttk.Label(outer, textvariable=self.stage_var, font=("Microsoft YaHei UI", 11)).pack(anchor="w")
        ttk.Label(outer, textvariable=self.title_var, font=("Microsoft YaHei UI", 20, "bold")).pack(anchor="w", pady=(12, 8))
        ttk.Label(
            outer,
            textvariable=self.instruction_var,
            font=("Microsoft YaHei UI", 13),
            wraplength=710,
            justify=LEFT,
        ).pack(anchor="w", pady=(0, 14))
        ttk.Label(outer, textvariable=self.countdown_var, font=("Microsoft YaHei UI", 18)).pack(anchor="center", pady=8)
        self.progress = ttk.Progressbar(outer, orient="horizontal", mode="determinate", maximum=100)
        self.progress.pack(fill="x", pady=(0, 16))

        buttons = ttk.Frame(outer)
        buttons.pack(fill="x")
        self.start_button = ttk.Button(buttons, text="开始当前阶段", command=self.start_state)
        self.start_button.pack(side=LEFT)
        ttk.Button(buttons, text="取消标定", command=self.close).pack(side=RIGHT)

        ttk.Separator(outer).pack(fill="x", pady=16)
        ttk.Label(outer, text="采集日志", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        self.log = __import__("tkinter").Text(outer, height=12, wrap="word", font=("Consolas", 10))
        self.log.pack(fill=BOTH, expand=True, pady=(6, 0))
        self.log.configure(state=DISABLED)
        self.show_state()
        self.write_log(f"监听 {host}:{port}，hand={hand}")
        self.write_log("WA100 接收器必须保持关闭；本窗口只读取实时 rotation。")
        root.after(100, self.poll_events)

    def write_log(self, message: str) -> None:
        self.log.configure(state=NORMAL)
        self.log.insert(END, message + "\n")
        self.log.see(END)
        self.log.configure(state=DISABLED)

    def show_state(self) -> None:
        if self.state_index >= len(STATES):
            self.stage_var.set("采集完成")
            self.title_var.set("正在计算标定参数")
            self.instruction_var.set("请保持窗口开启。只有通过质量门限的结果才会写入运行配置。")
            self.start_button.configure(state=DISABLED)
            self.countdown_var.set("计算中…")
            self.progress["value"] = 100
            threading.Thread(target=self.fit, daemon=True).start()
            return
        state = STATES[self.state_index]
        duration = self.duration_override if self.duration_override is not None else state.duration
        self.stage_var.set(f"阶段 {self.state_index + 1} / {len(STATES)}")
        self.title_var.set(state.label)
        self.instruction_var.set(state.instruction)
        self.countdown_var.set(f"准备好后点击开始，采集 {duration:g} 秒")
        self.progress["value"] = 0
        self.start_button.configure(state=NORMAL)

    def start_state(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        state = STATES[self.state_index]
        duration = self.duration_override if self.duration_override is not None else state.duration
        self.start_button.configure(state=DISABLED)
        self.deadline = time.monotonic() + duration
        self.write_log(f"开始：{state.label}")
        self.worker = threading.Thread(target=self.collect, args=(state.key, state.label, duration), daemon=True)
        self.worker.start()

    def collect(self, key: str, label: str, duration: float) -> None:
        frames: list[np.ndarray] = []
        last_frame = None
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline and not self.closed:
                try:
                    frame_id, rotations = receive_packet(self.sock, self.hand)
                except socket.timeout:
                    continue
                if frame_id >= 0 and last_frame is not None and frame_id <= last_frame:
                    continue
                last_frame = frame_id
                frames.append(rotations)
            minimum = max(20, int(duration * 10))
            if len(frames) < minimum:
                raise RuntimeError(f"{label} 仅收到 {len(frames)} 帧，至少需要 {minimum} 帧")
            self.events.put(("state_done", key, label, frames))
        except Exception as exc:
            if not self.closed:
                self.events.put(("error", str(exc)))

    def poll_events(self) -> None:
        if self.closed:
            return
        if self.worker is not None and self.worker.is_alive() and self.deadline > 0.0:
            state = STATES[self.state_index]
            duration = self.duration_override if self.duration_override is not None else state.duration
            remaining = max(0.0, self.deadline - time.monotonic())
            self.countdown_var.set(f"采集中：剩余 {remaining:4.1f} 秒")
            self.progress["value"] = 100.0 * (1.0 - remaining / max(duration, 1e-6))
        try:
            while True:
                event = self.events.get_nowait()
                kind = event[0]
                if kind == "state_done":
                    _, key, label, frames = event
                    self.samples[key] = frames
                    self.write_log(f"完成：{label}，{len(frames)} 帧")
                    self.state_index += 1
                    self.deadline = 0.0
                    self.show_state()
                elif kind == "error":
                    self.deadline = 0.0
                    self.start_button.configure(state=NORMAL)
                    self.countdown_var.set("采集失败，可检查后重试当前阶段")
                    self.write_log("错误：" + event[1])
                    messagebox.showerror("采集失败", event[1])
                elif kind == "fit_done":
                    self.handle_fit(event[1], event[2])
                elif kind == "fit_error":
                    self.write_log("拟合失败：" + event[1])
                    self.countdown_var.set("拟合失败")
                    messagebox.showerror("拟合失败", event[1])
        except queue.Empty:
            pass
        self.root.after(100, self.poll_events)

    def fit(self) -> None:
        try:
            save_samples(self.samples_path, self.samples)
            candidates = []
            failures = []
            for order in EULER_ORDERS:
                try:
                    candidates.append(fit_candidate(self.samples, order))
                except (ValueError, np.linalg.LinAlgError) as exc:
                    failures.append(str(exc))
            if not candidates:
                raise RuntimeError("所有欧拉角顺序均拟合失败：" + "; ".join(failures))
            candidates.sort(key=lambda item: (item["metrics"]["quality_pass"], item["selection_score"]), reverse=True)
            best = candidates[0]
            diagnostic_payload = {
                "selected_order": best["order"],
                "quality_pass": best["metrics"]["quality_pass"],
                "candidates": [
                    {"order": item["order"], "selection_score": item["selection_score"], "metrics": item["metrics"]}
                    for item in candidates
                ],
                "failures": failures,
            }
            self.diagnostics.parent.mkdir(parents=True, exist_ok=True)
            self.diagnostics.write_text(json.dumps(diagnostic_payload, ensure_ascii=False, indent=2), encoding="utf-8")
            if best["metrics"]["quality_pass"]:
                payload = calibration_payload(self.hand, best, {key: len(value) for key, value in self.samples.items()})
                self.output.parent.mkdir(parents=True, exist_ok=True)
                self.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            self.events.put(("fit_done", best, diagnostic_payload))
        except Exception as exc:
            self.events.put(("fit_error", str(exc)))

    def handle_fit(self, best: dict, diagnostics: dict) -> None:
        metrics = best["metrics"]
        passed = bool(metrics["quality_pass"])
        self.title_var.set("标定通过" if passed else "标定未通过")
        self.instruction_var.set(
            (f"运行配置已写入：{self.output}" if passed else "没有覆盖运行配置。请根据下列指标重新采集。")
        )
        self.countdown_var.set("完成")
        self.write_log(f"欧拉角顺序：{best['order']}")
        self.write_log(f"原始样本：{self.samples_path}")
        labels = (
            ("中立复测横摇漂移", "neutral_validation_yaw_target_range", 100, "≤"),
            ("中立复测弯曲漂移", "neutral_validation_pitch_target_range", 100, "≤"),
            ("横摇有效跨度", "yaw_sweep_target_range", 1600, "≥"),
            ("弯曲有效跨度", "pitch_sweep_target_range", 1600, "≥"),
            ("横摇时弯曲串扰", "yaw_sweep_pitch_crosstalk", 100, "≤"),
            ("弯曲时横摇串扰", "pitch_sweep_yaw_crosstalk", 100, "≤"),
        )
        for label, key, threshold, symbol in labels:
            self.write_log(f"{label}: {metrics[key]:.1f}（要求 {symbol}{threshold}）")
        if passed:
            messagebox.showinfo("标定通过", f"拇指横摇和弯曲配置已写入：\n{self.output}")
        else:
            messagebox.showwarning("标定未通过", f"未写入运行配置。诊断结果：\n{self.diagnostics}")

    def close(self) -> None:
        self.closed = True
        try:
            self.sock.close()
        except OSError:
            pass
        self.root.destroy()


def main() -> int:
    default_output = Path(__file__).resolve().parents[1] / "build" / "bin" / "thumb_glove_calibration.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--ip", default=UDP_IP)
    parser.add_argument("--port", type=int, default=UDP_PORT)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--diagnostics", type=Path)
    parser.add_argument("--samples", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    diagnostics = args.diagnostics.resolve() if args.diagnostics else output.with_name("thumb_glove_calibration_diagnostics.json")
    samples_path = args.samples.resolve() if args.samples else output.with_name("thumb_glove_calibration_samples.npz")
    root = Tk()
    try:
        CalibrationWindow(root, args.hand, args.ip, args.port, output, diagnostics, samples_path, args.duration)
    except OSError as exc:
        root.withdraw()
        messagebox.showerror("无法启动标定", f"不能监听 {args.ip}:{args.port}\n{exc}\n请先停止 WA100 接收器。")
        root.destroy()
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
