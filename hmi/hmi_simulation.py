from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List

import matplotlib.pyplot as plt


@dataclass
class HMIView:
    plc_status: str
    mode: str
    sequence_state: str
    conveyor_command: bool
    motor_feedback: bool
    product_count: int
    cycle_count: int
    fault_id: str
    fault_message: str
    alarms: List[str]
    scan_period_ms: float


def build_view(last_row: Dict[str, object]) -> HMIView:
    alarms = []
    if last_row.get("estop_latched"):
        alarms.append("Emergency stop active / recovery required")
    if last_row.get("fault_latched"):
        alarms.append(str(last_row.get("fault_message") or "PLC fault"))
    return HMIView(
        plc_status="FAULT" if last_row.get("fault_latched") else "RUNNING",
        mode=str(last_row.get("mode")),
        sequence_state=str(last_row.get("state")),
        conveyor_command=bool(last_row.get("conveyor_motor")),
        motor_feedback=bool(last_row.get("motor_feedback")),
        product_count=int(last_row.get("product_count", 0)),
        cycle_count=int(last_row.get("cycle_count", 0)),
        fault_id=str(last_row.get("fault_id", "")),
        fault_message=str(last_row.get("fault_message", "")),
        alarms=alarms,
        scan_period_ms=float(last_row.get("scan_period_s", 0.0)) * 1000.0,
    )


def render_dashboard(records: Iterable[Dict[str, object]], path: str) -> HMIView:
    rows = list(records)
    if not rows:
        raise ValueError("Cannot render an empty HMI log")
    last = rows[-1]
    view = build_view(last)
    t = [float(row["time_s"]) for row in rows]
    speed = [float(row["speed_mps"]) for row in rows]
    motor = [int(bool(row["conveyor_motor"])) for row in rows]
    alarms = [int(bool(row["alarm"])) for row in rows]

    fig = plt.figure(figsize=(13, 7), facecolor="#101820")
    grid = fig.add_gridspec(3, 3, height_ratios=[0.75, 1.25, 1.25], hspace=0.45, wspace=0.28)
    title = fig.add_subplot(grid[0, :])
    title.axis("off")
    title.text(0.01, 0.72, "CONVEYOR CELL / OPERATOR HMI", color="white", fontsize=20, weight="bold")
    title.text(0.01, 0.20, f"PLC: {view.plc_status}    MODE: {view.mode}    STATE: {view.sequence_state}", color="#7ee787", fontsize=12)
    cards = [
        ("CONVEYOR", "RUN" if view.conveyor_command else "STOP", "#7ee787" if view.conveyor_command else "#ffcc66"),
        ("MOTOR FB", "HEALTHY" if view.motor_feedback else "NOT READY", "#7ee787" if view.motor_feedback else "#ff7b72"),
        ("PRODUCTS", str(view.product_count), "#79c0ff"),
    ]
    for i, (label, value, color) in enumerate(cards):
        ax = fig.add_subplot(grid[1, i])
        ax.set_facecolor("#17232d")
        ax.axis("off")
        ax.text(0.08, 0.72, label, color="#aab8c2", fontsize=10)
        ax.text(0.08, 0.26, value, color=color, fontsize=22, weight="bold")

    trend = fig.add_subplot(grid[2, :2])
    trend.set_facecolor("#17232d")
    trend.plot(t, speed, color="#58a6ff", label="Measured speed (m/s)")
    trend.step(t, motor, where="post", color="#7ee787", alpha=0.7, label="Motor command")
    trend.set_xlabel("Time (s)", color="#aab8c2")
    trend.set_ylabel("Value", color="#aab8c2")
    trend.tick_params(colors="#aab8c2")
    trend.grid(alpha=0.15)
    trend.legend(frameon=False, labelcolor="white")
    alarms_ax = fig.add_subplot(grid[2, 2])
    alarms_ax.set_facecolor("#17232d")
    alarms_ax.step(t, alarms, where="post", color="#ff7b72")
    alarms_ax.set_title("Alarm history", color="white", fontsize=11)
    alarms_ax.set_xlabel("Time (s)", color="#aab8c2")
    alarms_ax.set_yticks([0, 1], ["clear", "active"], color="#aab8c2")
    alarms_ax.tick_params(axis="x", colors="#aab8c2")
    alarms_ax.grid(alpha=0.15)
    fig.text(0.72, 0.86, f"Cycle count: {view.cycle_count}\nScan: {view.scan_period_ms:.1f} ms\nFault: {view.fault_id or 'None'}", color="#d1d9e0", fontsize=10)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return view
