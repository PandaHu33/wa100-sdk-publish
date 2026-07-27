#!/usr/bin/env python3
"""Manual stage GUI for PICO controller and glove time-space alignment."""

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from tkinter import BOTH, DISABLED, END, LEFT, NORMAL, RIGHT, StringVar, Tk, messagebox
from tkinter import ttk

from glove_controller_alignment import (
    SideChannelCollector,
    fit_alignment,
    save_raw_samples,
)


@dataclass(frozen=True)
class Stage:
    key: str
    label: str
    instruction: str
    duration: float


STAGES = (
    Stage(
        "neutral",
        "中立静止",
        "前臂自然放置，手腕伸直，五指自然张开。姿势准备好后再点击开始，并保持完全静止。",
        3.0,
    ),
    Stage(
        "pivot",
        "腕心与时序采集",
        "把前臂稳固支撑在桌面上，五指保持不动。点击开始后，只绕腕关节中心缓慢做上下、左右和旋前三轴动作；不要移动整条前臂。",
        12.0,
    ),
)


class AlignmentWindow:
    def __init__(
        self,
        root: Tk,
        glove_port: int,
        controller_port: int,
        euler_order: str,
        output: Path,
        samples_path: Path,
        duration_scale: float,
    ):
        self.root = root
        self.euler_order = euler_order
        self.output = output
        self.samples_path = samples_path
        self.duration_scale = max(float(duration_scale), 0.1)
        self.collector = SideChannelCollector(glove_port, controller_port, euler_order)
        self.stage_index = 0
        self.stage_samples: dict[str, tuple[list, list]] = {}
        self.events: queue.Queue = queue.Queue()
        self.worker: threading.Thread | None = None
        self.deadline = 0.0
        self.closed = False

        root.title("手柄—手套时空标定")
        root.geometry("790x610")
        root.minsize(700, 540)
        root.attributes("-topmost", True)
        root.after(1500, lambda: root.attributes("-topmost", False))
        root.protocol("WM_DELETE_WINDOW", self.close)

        outer = ttk.Frame(root, padding=18)
        outer.pack(fill=BOTH, expand=True)
        self.stage_var = StringVar()
        self.title_var = StringVar()
        self.instruction_var = StringVar()
        self.status_var = StringVar(value="准备开始")
        ttk.Label(outer, textvariable=self.stage_var, font=("Microsoft YaHei UI", 11)).pack(anchor="w")
        ttk.Label(outer, textvariable=self.title_var, font=("Microsoft YaHei UI", 20, "bold")).pack(
            anchor="w", pady=(12, 8)
        )
        ttk.Label(
            outer,
            textvariable=self.instruction_var,
            font=("Microsoft YaHei UI", 13),
            wraplength=740,
            justify=LEFT,
        ).pack(anchor="w", pady=(0, 14))
        ttk.Label(outer, textvariable=self.status_var, font=("Microsoft YaHei UI", 17)).pack(
            anchor="center", pady=8
        )
        self.progress = ttk.Progressbar(outer, orient="horizontal", mode="determinate", maximum=100)
        self.progress.pack(fill="x", pady=(0, 16))

        buttons = ttk.Frame(outer)
        buttons.pack(fill="x")
        self.start_button = ttk.Button(buttons, text="开始当前阶段", command=self.start_stage)
        self.start_button.pack(side=LEFT)
        self.back_button = ttk.Button(buttons, text="重采上一阶段", command=self.redo_previous)
        self.back_button.pack(side=LEFT, padx=10)
        ttk.Button(buttons, text="取消标定", command=self.close).pack(side=RIGHT)

        ttk.Separator(outer).pack(fill="x", pady=16)
        ttk.Label(outer, text="采集与质量日志", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        self.log = __import__("tkinter").Text(outer, height=14, wrap="word", font=("Consolas", 10))
        self.log.pack(fill=BOTH, expand=True, pady=(6, 0))
        self.log.configure(state=DISABLED)
        self.write_log(f"监听手套 UDP {glove_port}，右手柄 UDP {controller_port}")
        self.write_log("本窗口只读，不发送机械臂或灵巧手命令；不要按 deadman。")
        self.show_stage()
        root.after(100, self.poll_events)

    def write_log(self, message: str) -> None:
        self.log.configure(state=NORMAL)
        self.log.insert(END, message + "\n")
        self.log.see(END)
        self.log.configure(state=DISABLED)

    def show_stage(self) -> None:
        if self.stage_index >= len(STAGES):
            self.stage_var.set("采集完成")
            self.title_var.set("正在计算时空标定")
            self.instruction_var.set("请保持窗口开启。质量不通过时不会覆盖上一份有效配置。")
            self.status_var.set("计算中…")
            self.start_button.configure(state=DISABLED)
            self.back_button.configure(state=DISABLED)
            self.progress["value"] = 100
            self.worker = threading.Thread(target=self.fit, daemon=True)
            self.worker.start()
            return
        stage = STAGES[self.stage_index]
        duration = stage.duration * self.duration_scale
        self.stage_var.set(f"阶段 {self.stage_index + 1} / {len(STAGES)}")
        self.title_var.set(stage.label)
        self.instruction_var.set(stage.instruction)
        self.status_var.set(f"准备好后手动点击开始，采集 {duration:g} 秒")
        self.progress["value"] = 0
        self.start_button.configure(state=NORMAL)
        self.back_button.configure(state=NORMAL if self.stage_index > 0 else DISABLED)

    def start_stage(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        stage = STAGES[self.stage_index]
        duration = stage.duration * self.duration_scale
        self.start_button.configure(state=DISABLED)
        self.back_button.configure(state=DISABLED)
        self.deadline = time.monotonic() + duration
        self.write_log(f"开始：{stage.label}")
        self.worker = threading.Thread(target=self.collect_stage, args=(stage, duration), daemon=True)
        self.worker.start()

    def collect_stage(self, stage: Stage, duration: float) -> None:
        try:
            controller, glove = self.collector.collect(duration)
            minimum_controller = max(15, int(duration * 10))
            minimum_glove = max(30, int(duration * 20))
            if len(controller) < minimum_controller or len(glove) < minimum_glove:
                raise RuntimeError(
                    f"{stage.label} 数据不足：手柄 {len(controller)}/{minimum_controller}，"
                    f"手套 {len(glove)}/{minimum_glove}"
                )
            self.events.put(("stage_done", stage, controller, glove))
        except Exception as exc:
            if not self.closed:
                self.events.put(("error", str(exc)))

    def redo_previous(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        if self.stage_index > 0:
            self.stage_index -= 1
            self.stage_samples.pop(STAGES[self.stage_index].key, None)
            self.write_log(f"返回重采：{STAGES[self.stage_index].label}")
            self.show_stage()

    def fit(self) -> None:
        try:
            neutral_controller, neutral_glove = self.stage_samples["neutral"]
            motion_controller, motion_glove = self.stage_samples["pivot"]
            save_raw_samples(
                self.samples_path,
                neutral_controller,
                neutral_glove,
                motion_controller,
                motion_glove,
            )
            payload = fit_alignment(
                neutral_controller,
                neutral_glove,
                motion_controller,
                motion_glove,
                self.euler_order,
            )
            self.output.parent.mkdir(parents=True, exist_ok=True)
            if payload["quality"]["quality_pass"]:
                self.output.write_text(
                    json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
                )
            else:
                failed = self.output.with_name(self.output.stem + ".failed.json")
                failed.write_text(
                    json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
                )
            self.events.put(("fit_done", payload))
        except Exception as exc:
            self.events.put(("fit_error", str(exc)))

    def poll_events(self) -> None:
        if self.closed:
            return
        if self.worker is not None and self.worker.is_alive() and self.deadline > 0:
            stage = STAGES[self.stage_index]
            duration = stage.duration * self.duration_scale
            remaining = max(0.0, self.deadline - time.monotonic())
            self.status_var.set(f"采集中：剩余 {remaining:4.1f} 秒")
            self.progress["value"] = 100.0 * (1.0 - remaining / max(duration, 1e-6))
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "stage_done":
                    _, stage, controller, glove = event
                    self.stage_samples[stage.key] = (controller, glove)
                    self.write_log(f"完成：{stage.label}，手柄 {len(controller)} 帧，手套 {len(glove)} 帧")
                    self.stage_index += 1
                    self.deadline = 0.0
                    self.show_stage()
                elif event[0] == "error":
                    self.deadline = 0.0
                    self.write_log("采集失败：" + event[1])
                    self.status_var.set("采集失败，可直接重试当前阶段")
                    self.start_button.configure(state=NORMAL)
                    messagebox.showerror("采集失败", event[1])
                elif event[0] == "fit_error":
                    self.deadline = 0.0
                    self.write_log("计算失败：" + event[1])
                    self.title_var.set("标定计算失败")
                    self.status_var.set("可关闭窗口后重新采集")
                    messagebox.showerror("标定失败", event[1])
                elif event[0] == "fit_done":
                    self.handle_fit(event[1])
        except queue.Empty:
            pass
        self.root.after(100, self.poll_events)

    def handle_fit(self, payload: dict) -> None:
        quality = payload["quality"]
        passed = bool(quality["quality_pass"])
        self.title_var.set("标定通过" if passed else "标定未通过")
        self.status_var.set("完成")
        self.instruction_var.set(
            f"有效配置已写入：{self.output}"
            if passed
            else "未覆盖有效配置。请查看指标，关闭窗口后重新采集。"
        )
        labels = (
            ("时间配对 P95", "paired_time_gap_p95_ms", "ms"),
            ("手柄运动幅度 P95", "controller_motion_p95_deg", "deg"),
            ("手套 Hand 运动 P95", "glove_hand_motion_p95_deg", "deg"),
            ("时间相关系数", "time_correlation_coefficient", ""),
            ("验证旋转残差中位数", "validation_rotation_residual_median_deg", "deg"),
            ("验证旋转残差 P95", "validation_rotation_residual_p95_deg", "deg"),
            ("腕心残差中位数", "pivot_residual_median_mm", "mm"),
            ("腕心残差 P95", "pivot_residual_p95_mm", "mm"),
            ("中立手柄角度波动 P95", "controller_orientation_p95_deg", "deg"),
            ("中立手套角度波动 P95", "glove_orientation_p95_deg", "deg"),
        )
        for label, key, unit in labels:
            if key in quality and isinstance(quality[key], (int, float)):
                self.write_log(f"{label}: {quality[key]:.3f} {unit}")
        self.write_log(f"质量结果：{'PASS' if passed else 'FAIL'}")
        self.write_log(f"原始样本：{self.samples_path}")
        if passed:
            messagebox.showinfo("标定通过", f"时空标定已写入：\n{self.output}")
        else:
            messagebox.showwarning(
                "标定未通过",
                f"未覆盖有效配置。诊断文件：\n{self.output.with_name(self.output.stem + '.failed.json')}",
            )

    def close(self) -> None:
        self.closed = True
        self.collector.close()
        self.root.destroy()


def main() -> int:
    default_output = Path(__file__).resolve().parents[1] / "build" / "bin" / "glove_controller_alignment.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--glove-port", type=int, default=25005)
    parser.add_argument("--controller-port", type=int, default=25006)
    parser.add_argument("--euler-order", default="zyx", choices=("xyz", "xzy", "yxz", "yzx", "zxy", "zyx"))
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--samples", type=Path)
    parser.add_argument("--duration-scale", type=float, default=1.0)
    args = parser.parse_args()
    output = args.output.resolve()
    samples = args.samples.resolve() if args.samples else output.with_name("glove_controller_alignment_samples.npz")
    root = Tk()
    try:
        AlignmentWindow(
            root,
            args.glove_port,
            args.controller_port,
            args.euler_order,
            output,
            samples,
            args.duration_scale,
        )
    except OSError as exc:
        root.withdraw()
        messagebox.showerror("无法启动标定", f"无法监听标定旁路端口：\n{exc}")
        root.destroy()
        return 1
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
