#!/usr/bin/env python3
"""从 PX4 ULog 提取差速 Rover 调参图表和结构化摘要。

脚本只向 pyulog 请求调参所需 topic，不展开或导出整份日志原始数据。

用法：
  python3 rover_ulog_plot.py log.ulg
  python3 rover_ulog_plot.py log.ulg --out /tmp/rover-report
  python3 rover_ulog_plot.py log.ulg --start 30 --end 90
  python3 rover_ulog_plot.py log.ulg --list
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path
from typing import Iterable, Sequence


os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rover-ulog")
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from pyulog import ULog  # noqa: E402

plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "WenQuanYi Micro Hei",
    "Noto Serif CJK SC",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


TOPIC_FILTER = [
    "actuator_armed",
    "actuator_motors",
    "actuator_motors_rover",
    "differential_velocity_setpoint",
    "hybrid_vehicle_status",
    "m2006_motor_status",
    "manual_control_setpoint",
    "offboard_control_mode",
    "position_setpoint_triplet",
    "pure_pursuit_status",
    "rover_attitude_setpoint",
    "rover_attitude_status",
    "rover_position_setpoint",
    "rover_rate_setpoint",
    "rover_rate_status",
    "rover_steering_setpoint",
    "rover_throttle_setpoint",
    "rover_velocity_status",
    "vehicle_angular_velocity",
    "vehicle_attitude",
    "vehicle_control_mode",
    "vehicle_local_position",
    "vehicle_status",
]

ROVER_PARAM_PREFIXES = ("RO_", "RD_", "PP_", "M2K_")
ROVER_PARAM_NAMES = {"CA_R_REV", "NAV_ACC_RAD"}

FIELD_ALIASES = {
    ("manual_control_setpoint", "roll"): ("roll", "x"),
    ("manual_control_setpoint", "pitch"): ("pitch", "y"),
    ("manual_control_setpoint", "yaw"): ("yaw", "r"),
    ("manual_control_setpoint", "throttle"): ("throttle", "z"),
}


class LogData:
    def __init__(self, ulog: ULog, start_s: float | None, end_s: float | None) -> None:
        self.ulog = ulog
        self.topics = {}

        for data in ulog.data_list:
            previous = self.topics.get(data.name)

            if previous is None or getattr(data, "multi_id", 0) == 0:
                self.topics[data.name] = data

        starts = [
            int(data.data["timestamp"][0])
            for data in self.topics.values()
            if "timestamp" in data.data and len(data.data["timestamp"]) > 0
        ]
        self.t0 = min(starts) if starts else 0
        self.start_s = start_s
        self.end_s = end_s

    def has_topic(self, topic: str) -> bool:
        return topic in self.topics

    def resolve_field(self, topic: str, field: str) -> str | None:
        if topic not in self.topics:
            return None

        for candidate in FIELD_ALIASES.get((topic, field), (field,)):
            if candidate in self.topics[topic].data:
                return candidate

        return None

    def series(self, topic: str, field: str) -> tuple[np.ndarray, np.ndarray] | None:
        actual_field = self.resolve_field(topic, field)

        if actual_field is None or "timestamp" not in self.topics[topic].data:
            return None

        timestamp = self.topics[topic].data["timestamp"].astype(float)
        time_s = (timestamp - float(self.t0)) * 1e-6
        values = self.topics[topic].data[actual_field].astype(float)
        mask = np.ones(time_s.shape, dtype=bool)

        if self.start_s is not None:
            mask &= time_s >= self.start_s

        if self.end_s is not None:
            mask &= time_s <= self.end_s

        return time_s[mask], values[mask]


def plot_series(
    ax: plt.Axes,
    log: LogData,
    specs: Iterable[tuple[str, str, str, float]],
    missing: set[str],
) -> bool:
    plotted = False

    for topic, field, label, scale in specs:
        series = log.series(topic, field)

        if series is None:
            missing.add(f"{topic}.{field}")
            continue

        time_s, values = series

        if len(time_s) == 0:
            continue

        ax.plot(time_s, values * scale, label=label, linewidth=1.1)
        plotted = True

    if plotted:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    return plotted


def save_figure(fig: plt.Figure, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_rate_loop(log: LogData, out_dir: Path, missing: set[str]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("Rover 偏航角速度内环")
    plot_series(
        axes[0],
        log,
        [
            ("rover_rate_setpoint", "yaw_rate_setpoint", "原始角速度设定 [rad/s]", 1.0),
            ("rover_rate_status", "adjusted_yaw_rate_setpoint", "斜率限制后设定 [rad/s]", 1.0),
            ("rover_rate_status", "measured_yaw_rate", "控制器实测 [rad/s]", 1.0),
            ("vehicle_angular_velocity", "xyz[2]", "gyro Z [rad/s]", 1.0),
        ],
        missing,
    )
    axes[0].set_ylabel("yaw rate")
    plot_series(
        axes[1],
        log,
        [
            ("rover_steering_setpoint", "normalized_speed_diff", "归一化差速", 1.0),
            ("rover_rate_status", "pid_yaw_rate_integral", "角速度积分项", 1.0),
        ],
        missing,
    )
    axes[1].set_ylabel("controller")
    plot_series(
        axes[2],
        log,
        [
            ("manual_control_setpoint", "roll", "遥控 roll", 1.0),
            ("actuator_motors_rover", "control[0]", "Rover 左源输出", 1.0),
            ("actuator_motors_rover", "control[1]", "Rover 右源输出", 1.0),
            ("actuator_motors", "control[5]", "最终左轮 Motor 6", 1.0),
            ("actuator_motors", "control[4]", "最终右轮 Motor 5", 1.0),
        ],
        missing,
    )
    axes[2].set_ylabel("output")
    axes[2].set_xlabel("time [s]")
    save_figure(fig, out_dir / "01_rover_yaw_rate_loop.png")


def save_attitude_loop(log: LogData, out_dir: Path, missing: set[str]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("Rover 航向外环与角速度串级")
    plot_series(
        axes[0],
        log,
        [
            ("rover_attitude_setpoint", "yaw_setpoint", "原始航向设定 [rad]", 1.0),
            ("rover_attitude_status", "adjusted_yaw_setpoint", "调整后航向设定 [rad]", 1.0),
            ("rover_attitude_status", "measured_yaw", "实测航向 [rad]", 1.0),
            ("vehicle_local_position", "heading", "local heading [rad]", 1.0),
        ],
        missing,
    )
    axes[0].set_ylabel("yaw")
    plot_series(
        axes[1],
        log,
        [
            ("rover_rate_setpoint", "yaw_rate_setpoint", "外环输出角速度 [rad/s]", 1.0),
            ("rover_rate_status", "adjusted_yaw_rate_setpoint", "调整后角速度 [rad/s]", 1.0),
            ("rover_rate_status", "measured_yaw_rate", "实测角速度 [rad/s]", 1.0),
        ],
        missing,
    )
    axes[1].set_ylabel("yaw rate")
    plot_series(
        axes[2],
        log,
        [
            ("rover_steering_setpoint", "normalized_speed_diff", "归一化差速", 1.0),
            ("manual_control_setpoint", "roll", "遥控 roll", 1.0),
            ("manual_control_setpoint", "pitch", "遥控 pitch", 1.0),
        ],
        missing,
    )
    axes[2].set_ylabel("control")
    axes[2].set_xlabel("time [s]")
    save_figure(fig, out_dir / "02_rover_yaw_attitude_loop.png")


def save_speed_loop(log: LogData, out_dir: Path, missing: set[str]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("Rover 车体前向速度环")
    plot_series(
        axes[0],
        log,
        [
            ("differential_velocity_setpoint", "speed", "原始速度设定 [m/s]", 1.0),
            ("rover_velocity_status", "speed_body_x_setpoint", "控制器速度设定 [m/s]", 1.0),
            ("rover_velocity_status", "adjusted_speed_body_x_setpoint", "斜率限制后设定 [m/s]", 1.0),
            ("rover_velocity_status", "measured_speed_body_x", "实测 body-X [m/s]", 1.0),
        ],
        missing,
    )
    axes[0].set_ylabel("speed")
    plot_series(
        axes[1],
        log,
        [
            ("rover_throttle_setpoint", "throttle_body_x", "归一化油门设定", 1.0),
            ("rover_velocity_status", "pid_throttle_body_x_integral", "速度积分项", 1.0),
            ("manual_control_setpoint", "pitch", "遥控 pitch", 1.0),
        ],
        missing,
    )
    axes[1].set_ylabel("throttle")
    plot_series(
        axes[2],
        log,
        [
            ("actuator_motors_rover", "control[0]", "Rover 左源输出", 1.0),
            ("actuator_motors_rover", "control[1]", "Rover 右源输出", 1.0),
            ("actuator_motors", "control[5]", "最终左轮 Motor 6", 1.0),
            ("actuator_motors", "control[4]", "最终右轮 Motor 5", 1.0),
            ("rover_steering_setpoint", "normalized_speed_diff", "归一化差速", 1.0),
        ],
        missing,
    )
    axes[2].set_ylabel("output")
    axes[2].set_xlabel("time [s]")
    save_figure(fig, out_dir / "03_rover_speed_loop.png")


def save_path_tracking(log: LogData, out_dir: Path, missing: set[str]) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(13, 9))
    fig.suptitle("Rover 路径与 Pure Pursuit")
    plotted_xy = False
    x_series = log.series("vehicle_local_position", "x")
    y_series = log.series("vehicle_local_position", "y")

    if x_series is not None and y_series is not None:
        count = min(len(x_series[1]), len(y_series[1]))

        if count > 0:
            axes[0].plot(y_series[1][:count], x_series[1][:count], label="vehicle path", linewidth=1.2)
            axes[0].invert_yaxis()
            axes[0].axis("equal")
            axes[0].grid(True, alpha=0.3)
            axes[0].legend(loc="best")
            plotted_xy = True

    if not plotted_xy:
        missing.add("vehicle_local_position.x/y")

    axes[0].set_xlabel("East [m]")
    axes[0].set_ylabel("North [m]")
    plot_series(
        axes[1],
        log,
        [
            ("pure_pursuit_status", "crosstrack_error", "横向误差 [m]", 1.0),
            ("pure_pursuit_status", "lookahead_distance", "前视距离 [m]", 1.0),
            ("pure_pursuit_status", "distance_to_waypoint", "航点距离 [m]", 1.0),
        ],
        missing,
    )
    axes[1].set_ylabel("distance")
    axes[1].set_xlabel("time [s]")
    save_figure(fig, out_dir / "04_rover_path_tracking.png")


def save_m2006_loop(log: LogData, out_dir: Path, missing: set[str]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("M2006/C610 左右轮最内层")
    plot_series(
        axes[0],
        log,
        [
            ("m2006_motor_status", "target_rpm[0]", "左目标 rotor rpm", 1.0),
            ("m2006_motor_status", "measured_rpm[0]", "左实测 rotor rpm", 1.0),
            ("m2006_motor_status", "target_rpm[1]", "右目标 rotor rpm", 1.0),
            ("m2006_motor_status", "measured_rpm[1]", "右实测 rotor rpm", 1.0),
        ],
        missing,
    )
    axes[0].set_ylabel("rotor rpm")
    plot_series(
        axes[1],
        log,
        [
            ("m2006_motor_status", "current_command[0]", "左电流命令", 1.0),
            ("m2006_motor_status", "torque_current[0]", "左反馈电流", 1.0),
            ("m2006_motor_status", "current_command[1]", "右电流命令", 1.0),
            ("m2006_motor_status", "torque_current[1]", "右反馈电流", 1.0),
        ],
        missing,
    )
    axes[1].set_ylabel("current unit")
    plot_series(
        axes[2],
        log,
        [
            ("m2006_motor_status", "fault_flags", "fault flags", 1.0),
            ("m2006_motor_status", "online[0]", "left online", 1.0),
            ("m2006_motor_status", "online[1]", "right online", 1.0),
        ],
        missing,
    )
    axes[2].set_ylabel("health")
    axes[2].set_xlabel("time [s]")
    save_figure(fig, out_dir / "05_m2006_speed_loops.png")


def save_mode_overview(log: LogData, out_dir: Path, missing: set[str]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("Rover 模式与控制环启用状态")
    plot_series(
        axes[0],
        log,
        [
            ("vehicle_status", "nav_state", "nav state", 1.0),
            ("vehicle_status", "vehicle_type", "vehicle type", 1.0),
            ("hybrid_vehicle_status", "current_state", "hybrid state", 1.0),
        ],
        missing,
    )
    axes[0].set_ylabel("state enum")
    plot_series(
        axes[1],
        log,
        [
            ("vehicle_control_mode", "flag_control_rates_enabled", "rate loop", 1.0),
            ("vehicle_control_mode", "flag_control_attitude_enabled", "attitude loop", 1.0),
            ("vehicle_control_mode", "flag_control_velocity_enabled", "velocity loop", 1.0),
            ("vehicle_control_mode", "flag_control_position_enabled", "position loop", 1.0),
            ("vehicle_control_mode", "flag_control_auto_enabled", "auto", 1.0),
            ("vehicle_control_mode", "flag_control_offboard_enabled", "offboard", 1.0),
        ],
        missing,
    )
    axes[1].set_ylabel("enabled")
    axes[1].set_ylim(-0.1, 1.1)
    plot_series(
        axes[2],
        log,
        [
            ("actuator_armed", "armed", "armed", 1.0),
            ("actuator_armed", "lockdown", "lockdown", 1.0),
            ("actuator_armed", "force_failsafe", "force failsafe", 1.0),
            ("vehicle_status", "failsafe", "vehicle failsafe", 1.0),
        ],
        missing,
    )
    axes[2].set_ylabel("safety")
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_xlabel("time [s]")
    save_figure(fig, out_dir / "06_rover_mode_overview.png")


def finite_rms(values: np.ndarray) -> float | None:
    finite = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(np.square(finite)))) if finite.size else None


def aligned_error(log: LogData, setpoint: tuple[str, str], measured: tuple[str, str]) -> np.ndarray | None:
    sp = log.series(*setpoint)
    meas = log.series(*measured)

    if sp is None or meas is None or len(sp[0]) < 2 or len(meas[0]) == 0:
        return None

    valid = np.isfinite(sp[0]) & np.isfinite(sp[1])

    if np.count_nonzero(valid) < 2:
        return None

    interpolated = np.interp(meas[0], sp[0][valid], sp[1][valid])
    return interpolated - meas[1]


def metric_line(label: str, value: float | None, unit: str) -> str:
    return f"- {label}: {value:.4f} {unit}" if value is not None else f"- {label}: unavailable"


def parameter_lines(ulog: ULog) -> list[str]:
    params = {
        name: value
        for name, value in getattr(ulog, "initial_parameters", {}).items()
        if name.startswith(ROVER_PARAM_PREFIXES) or name in ROVER_PARAM_NAMES
    }
    return [f"- `{name}` = `{params[name]}`" for name in sorted(params)] or ["- none"]


def write_summary(log: LogData, out_dir: Path, missing: set[str]) -> None:
    rate_error = aligned_error(
        log,
        ("rover_rate_status", "adjusted_yaw_rate_setpoint"),
        ("rover_rate_status", "measured_yaw_rate"),
    )
    speed_error = aligned_error(
        log,
        ("rover_velocity_status", "adjusted_speed_body_x_setpoint"),
        ("rover_velocity_status", "measured_speed_body_x"),
    )
    left_error = aligned_error(log, ("m2006_motor_status", "target_rpm[0]"), ("m2006_motor_status", "measured_rpm[0]"))
    right_error = aligned_error(log, ("m2006_motor_status", "target_rpm[1]"), ("m2006_motor_status", "measured_rpm[1]"))
    fault_series = log.series("m2006_motor_status", "fault_flags")
    fault_samples = 0

    if fault_series is not None:
        fault_samples = int(np.count_nonzero(np.nan_to_num(fault_series[1], nan=0.0) != 0.0))

    lines = [
        "# Rover ULog 调参摘要",
        "",
        "## 分析范围",
        "",
        f"- 起始时间: {log.start_s if log.start_s is not None else '日志起点'} s",
        f"- 结束时间: {log.end_s if log.end_s is not None else '日志终点'} s",
        "- 仅加载预定义 Rover 调参 topic；未导出原始逐样本数据。",
        "",
        "## 核心误差指标",
        "",
        metric_line("偏航角速度 RMS 误差", finite_rms(rate_error) if rate_error is not None else None, "rad/s"),
        metric_line("车体前向速度 RMS 误差", finite_rms(speed_error) if speed_error is not None else None, "m/s"),
        metric_line("左 M2006 RMS 转速误差", finite_rms(left_error) if left_error is not None else None, "rpm"),
        metric_line("右 M2006 RMS 转速误差", finite_rms(right_error) if right_error is not None else None, "rpm"),
        f"- M2006 非零故障样本数: {fault_samples}",
        "",
        "这些指标必须结合图中有效激励、饱和、静摩擦区和模式区间解释，不能单独作为调参结论。",
        "",
        "## 日志内 Rover 参数快照",
        "",
        *parameter_lines(log.ulog),
        "",
        "## 已加载的相关 Topic",
        "",
    ]

    for topic in TOPIC_FILTER:
        if topic in log.topics:
            lines.append(f"- `{topic}`: {len(log.topics[topic].data.get('timestamp', []))} samples")

    lines.extend(["", "## 缺失字段", ""])
    lines.extend([f"- `{item}`" for item in sorted(missing)] if missing else ["- none"])
    lines.extend(
        [
            "",
            "## 输出图",
            "",
            "- `01_rover_yaw_rate_loop.png`",
            "- `02_rover_yaw_attitude_loop.png`",
            "- `03_rover_speed_loop.png`",
            "- `04_rover_path_tracking.png`",
            "- `05_m2006_speed_loops.png`",
            "- `06_rover_mode_overview.png`",
            "",
        ]
    )
    (out_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def list_relevant_topics(log: LogData) -> None:
    for topic in TOPIC_FILTER:
        if topic in log.topics:
            fields = ", ".join(sorted(log.topics[topic].data.keys()))
            print(f"{topic}: {fields}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成 PX4 差速 Rover 控制环调参图表和摘要。")
    parser.add_argument("ulg", help="输入 .ulg 文件")
    parser.add_argument("--out", help="输出目录；默认 rover_ulog_plots/<日志名>")
    parser.add_argument("--start", type=float, help="分析起始时间，单位为日志起点后的秒")
    parser.add_argument("--end", type=float, help="分析结束时间，单位为日志起点后的秒")
    parser.add_argument("--list", action="store_true", help="只列出已筛选 topic 的可用字段")
    args = parser.parse_args(argv)

    if args.start is not None and args.start < 0:
        parser.error("--start 不能小于 0")

    if args.start is not None and args.end is not None and args.end <= args.start:
        parser.error("--end 必须大于 --start")

    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    ulg_path = Path(args.ulg).expanduser().resolve()

    if not ulg_path.is_file():
        print(f"error: ULog 不存在: {ulg_path}", file=sys.stderr)
        return 2

    try:
        ulog = ULog(str(ulg_path), message_name_filter_list=TOPIC_FILTER)
    except Exception as exc:
        print(f"error: 读取 ULog 失败: {exc}", file=sys.stderr)
        return 3

    log = LogData(ulog, args.start, args.end)

    if args.list:
        list_relevant_topics(log)
        return 0

    out_dir = Path(args.out).expanduser().resolve() if args.out else Path("rover_ulog_plots") / ulg_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    missing: set[str] = set()
    save_rate_loop(log, out_dir, missing)
    save_attitude_loop(log, out_dir, missing)
    save_speed_loop(log, out_dir, missing)
    save_path_tracking(log, out_dir, missing)
    save_m2006_loop(log, out_dir, missing)
    save_mode_overview(log, out_dir, missing)
    write_summary(log, out_dir, missing)
    print(f"已生成: {out_dir}")
    print("包含 6 张控制环/模式图和 summary.md；未导出原始逐样本数据。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
