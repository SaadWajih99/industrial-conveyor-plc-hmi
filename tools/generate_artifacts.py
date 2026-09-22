from __future__ import annotations

import json
import shutil
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import FancyBboxPatch

ROOT = Path(__file__).resolve().parents[1]
for directory in [ROOT / "data", ROOT / "results", ROOT / "diagrams", ROOT / "analysis", ROOT / "docs"]:
    directory.mkdir(parents=True, exist_ok=True)

sys.path.insert(0, str(ROOT))
from control.pid_tuning import (  # noqa: E402
    MotorModel,
    manual_tuning,
    optimize_tuning,
    simulate_controller,
    ziegler_nichols_tuning,
)
from hmi.hmi_simulation import render_dashboard  # noqa: E402
from simulation.testbench import ScenarioConfig, run_scenario  # noqa: E402


def write_notebook(path: Path, cells: list[tuple[str, str]]) -> None:
    notebook = {
        "cells": [
            {"cell_type": kind, "metadata": {}, "source": source.splitlines(True), **({"outputs": [], "execution_count": None} if kind == "code" else {})}
            for kind, source in cells
        ],
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python", "version": "3"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }
    path.write_text(json.dumps(notebook, indent=2), encoding="utf-8")


def draw_box(ax, xy, width, height, text, color="#16324F", text_color="white", fontsize=10):
    box = FancyBboxPatch(xy, width, height, boxstyle="round,pad=0.02,rounding_size=0.02", linewidth=1.3, edgecolor="#7ea6c7", facecolor=color)
    ax.add_patch(box)
    ax.text(xy[0] + width / 2, xy[1] + height / 2, text, ha="center", va="center", color=text_color, fontsize=fontsize, weight="bold", wrap=True)


def arrow(ax, start, end, color="#5BC0EB", text=None, text_offset=(0, 0)):
    ax.annotate("", xy=end, xytext=start, arrowprops={"arrowstyle": "->", "lw": 1.8, "color": color})
    if text:
        ax.text((start[0] + end[0]) / 2 + text_offset[0], (start[1] + end[1]) / 2 + text_offset[1], text, color=color, fontsize=8, ha="center", va="center")


def architecture_diagram(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(14, 7), facecolor="#0b1220")
    ax.set_facecolor("#0b1220")
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.axis("off")
    ax.text(0.4, 7.55, "PLC-BASED CONVEYOR CELL / SYSTEM ARCHITECTURE", color="white", fontsize=17, weight="bold")
    draw_box(ax, (0.5, 4.7), 2.0, 1.2, "FIELD INPUTS\nphotoelectric sensors\nlimits / feedback", "#1e4d59")
    draw_box(ax, (3.1, 4.7), 2.0, 1.2, "PLC INPUT IMAGE\nI0.x / I1.x\nNC safety signal", "#215273")
    draw_box(ax, (5.7, 4.3), 2.7, 2.0, "PLC CONTROL LOGIC\nscan cycle\ninterlocks\ntimers + counters\nstate machine", "#503b6b")
    draw_box(ax, (8.9, 4.7), 2.0, 1.2, "PLC OUTPUT IMAGE\nQ0.x\nsafe-state gating", "#215273")
    draw_box(ax, (11.5, 4.7), 2.0, 1.2, "ACTUATORS\nmotor / VFD\ndiverter / lamps", "#1e4d59")
    arrow(ax, (2.5, 5.3), (3.1, 5.3), text="field wiring")
    arrow(ax, (5.1, 5.3), (5.7, 5.3), text="read")
    arrow(ax, (8.4, 5.3), (8.9, 5.3), text="write")
    arrow(ax, (10.9, 5.3), (11.5, 5.3), text="driver")
    draw_box(ax, (5.0, 1.4), 3.8, 1.1, "HMI / OPERATOR\nmode • commands • status • alarms • trends", "#6a4e35")
    arrow(ax, (6.9, 4.3), (6.9, 2.5), color="#f6c85f", text="commands / status", text_offset=(0.65, 0))
    draw_box(ax, (0.8, 1.4), 2.8, 1.1, "SAFETY PATH\nE-stop NC input\nphysical safety circuit boundary", "#6b2f36")
    arrow(ax, (3.6, 1.95), (5.0, 1.95), color="#ff7b72", text="permissive")
    arrow(ax, (5.0, 2.2), (6.2, 4.3), color="#ff7b72", text="inhibits motion", text_offset=(0.45, 0))
    ax.text(0.5, 0.45, "Simulation boundary: software model only; physical safety implementation requires independent safety-rated design and validation.", color="#ffcc66", fontsize=9)
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def control_loop_diagram(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(13, 4.6), facecolor="#0b1220")
    ax.set_facecolor("#0b1220")
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 5)
    ax.axis("off")
    ax.text(0.4, 4.55, "CLOSED-LOOP CONVEYOR SPEED CONTROL", color="white", fontsize=16, weight="bold")
    draw_box(ax, (0.5, 2.0), 1.6, 1.0, "Reference\n0.8 m/s", "#1e4d59")
    draw_box(ax, (2.8, 2.0), 1.6, 1.0, "Error\ne = r - y", "#503b6b")
    draw_box(ax, (5.1, 2.0), 1.8, 1.0, "PID\nKp + Ki + Kd", "#503b6b")
    draw_box(ax, (7.6, 2.0), 1.8, 1.0, "Motor / VFD\n0..100% command", "#215273")
    draw_box(ax, (10.1, 2.0), 1.8, 1.0, "Plant\nconveyor speed", "#1e4d59")
    arrow(ax, (2.1, 2.5), (2.8, 2.5))
    arrow(ax, (4.4, 2.5), (5.1, 2.5))
    arrow(ax, (6.9, 2.5), (7.6, 2.5))
    arrow(ax, (9.4, 2.5), (10.1, 2.5))
    ax.annotate("", xy=(3.6, 1.5), xytext=(11.0, 1.5), arrowprops={"arrowstyle": "->", "lw": 1.7, "color": "#f6c85f"})
    ax.text(7.3, 1.25, "measured speed feedback", color="#f6c85f", fontsize=9, ha="center")
    ax.annotate("", xy=(2.85, 1.5), xytext=(3.6, 1.5), arrowprops={"arrowstyle": "-", "lw": 1.7, "color": "#f6c85f"})
    ax.text(2.05, 3.4, "anti-windup + filtered derivative", color="#aab8c2", fontsize=9)
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def state_machine_diagram(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 7), facecolor="#0b1220")
    ax.set_facecolor("#0b1220")
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.text(0.35, 6.6, "AUTOMATIC SEQUENCE STATE MACHINE", color="white", fontsize=16, weight="bold")
    positions = {
        "IDLE": (0.7, 4.4), "STARTUP": (3.0, 4.4), "CONVEYING": (5.4, 4.4),
        "PRODUCT_DETECTED": (8.0, 4.4), "POSITIONING": (2.0, 2.1),
        "SORTING": (5.0, 2.1), "PRODUCT_EXIT": (8.0, 2.1),
    }
    for name, xy in positions.items():
        draw_box(ax, xy, 1.65, 0.78, name.replace("_", "\n"), "#503b6b" if name in {"SORTING", "POSITIONING"} else "#215273", fontsize=8)
    edges = [("IDLE", "STARTUP", "AUTO + RUN"), ("STARTUP", "CONVEYING", "motor FB"), ("CONVEYING", "PRODUCT_DETECTED", "entry"), ("PRODUCT_DETECTED", "POSITIONING", "position"), ("POSITIONING", "SORTING", "sort latch"), ("POSITIONING", "PRODUCT_EXIT", "unsorted + exit"), ("SORTING", "PRODUCT_EXIT", "exit"), ("PRODUCT_EXIT", "CONVEYING", "exit clear")]
    for source, target, label in edges:
        sx, sy = positions[source]
        tx, ty = positions[target]
        arrow(ax, (sx + 0.82, sy + 0.39), (tx + 0.82, ty + 0.39), color="#5BC0EB", text=label, text_offset=(0, 0.13))
    draw_box(ax, (0.8, 0.55), 2.0, 0.7, "FAULT", "#6b2f36")
    draw_box(ax, (4.0, 0.55), 2.0, 0.7, "EMERGENCY_STOP", "#6b2f36")
    draw_box(ax, (7.2, 0.55), 2.0, 0.7, "RESET", "#6a4e35")
    ax.text(9.7, 0.8, "Any state -> FAULT or\nEMERGENCY_STOP on unsafe condition", color="#ffcc66", fontsize=9)
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def wiring_diagram(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 6), facecolor="#0b1220")
    ax.set_facecolor("#0b1220")
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6)
    ax.axis("off")
    ax.text(0.35, 5.55, "CONCEPTUAL 24 VDC FIELD WIRING", color="white", fontsize=16, weight="bold")
    draw_box(ax, (0.5, 3.6), 1.6, 1.0, "24 VDC\nprotected supply", "#6a4e35")
    draw_box(ax, (3.0, 3.6), 2.1, 1.0, "Sensors\nPNP / photoelectric\n0 V common", "#1e4d59")
    draw_box(ax, (6.0, 3.35), 2.1, 1.5, "PLC\nDI / DO\nCPU + logic", "#503b6b")
    draw_box(ax, (9.0, 3.6), 2.1, 1.0, "Interface\nrelay / driver / VFD", "#215273")
    draw_box(ax, (9.0, 1.2), 2.1, 1.0, "Motor + diverter\nactuators", "#1e4d59")
    arrow(ax, (2.1, 4.1), (3.0, 4.1), color="#f6c85f", text="24 V")
    arrow(ax, (5.1, 4.1), (6.0, 4.1), text="DI")
    arrow(ax, (8.1, 4.1), (9.0, 4.1), text="DO")
    arrow(ax, (10.05, 3.6), (10.05, 2.2), text="isolated drive")
    draw_box(ax, (0.7, 1.25), 2.3, 0.9, "E-stop circuit\nNC contacts / safety relay\nindependent energy removal", "#6b2f36", fontsize=9)
    arrow(ax, (3.0, 1.7), (6.0, 3.35), color="#ff7b72", text="status / permissive", text_offset=(0.2, 0))
    ax.text(0.5, 0.35, "Design considerations: fused branches, 0 V reference, shield/ground practice, isolation for inductive loads, and validated STO/safety circuit in hardware.", color="#aab8c2", fontsize=8.5)
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def io_diagram(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(13, 6), facecolor="#0b1220")
    ax.set_facecolor("#0b1220")
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 6)
    ax.axis("off")
    ax.text(0.35, 5.55, "PLC I/O PARTITION", color="white", fontsize=16, weight="bold")
    draw_box(ax, (0.6, 1.0), 3.0, 4.0, "DIGITAL INPUTS\n\nI0.0  E-stop healthy\nI0.1  Start\nI0.2  Stop\nI0.3  Reset\nI0.4  Entry sensor\nI0.5  Position sensor\nI0.6  Exit sensor\nI0.7  Motor feedback\nI1.0  Diverter home\nI1.1  Diverter extended\nI1.2  Sort request", "#1e4d59", fontsize=9)
    draw_box(ax, (4.9, 1.0), 3.2, 4.0, "PLC PROCESSING\n\ninput image\nmode selection\npermissive\nseal-in latch\nTON timers\nedge counters\nstate machine\nfault latch\noutput image", "#503b6b", fontsize=10)
    draw_box(ax, (9.4, 1.0), 3.0, 4.0, "DIGITAL OUTPUTS\n\nQ0.0 Motor command\nQ0.1 Diverter extend\nQ0.2 Diverter retract\nQ0.3 Green lamp\nQ0.4 Red fault lamp\nQ0.5 Alarm", "#215273", fontsize=10)
    arrow(ax, (3.6, 3.0), (4.9, 3.0), text="read")
    arrow(ax, (8.1, 3.0), (9.4, 3.0), text="write")
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def drawio_file(path: Path) -> None:
    xml = """<mxfile host="app.diagrams.net" modified="2026-09-22T00:00:00.000Z" agent="browser-conveyor-project" version="24.7.17"><diagram id="architecture" name="System Architecture"><mxGraphModel><root><mxCell id="0"/><mxCell id="1" parent="0"/><mxCell id="sensors" value="Field Inputs&#xa;Sensors / E-stop" style="rounded=1;whiteSpace=wrap;html=1;fillColor=#d5e8d4;strokeColor=#82b366" vertex="1" parent="1"><mxGeometry x="40" y="180" width="160" height="80" as="geometry"/></mxCell><mxCell id="plc" value="PLC Logic&#xa;Scan / Interlocks / Sequence" style="rounded=1;whiteSpace=wrap;html=1;fillColor=#e1d5e7;strokeColor=#9673a6" vertex="1" parent="1"><mxGeometry x="300" y="180" width="210" height="80" as="geometry"/></mxCell><mxCell id="act" value="Actuators&#xa;Motor / Diverter" style="rounded=1;whiteSpace=wrap;html=1;fillColor=#dae8fc;strokeColor=#6c8ebf" vertex="1" parent="1"><mxGeometry x="620" y="180" width="170" height="80" as="geometry"/></mxCell><mxCell id="hmi" value="HMI&#xa;Commands / Alarms / Trends" style="rounded=1;whiteSpace=wrap;html=1;fillColor=#fff2cc;strokeColor=#d6b656" vertex="1" parent="1"><mxGeometry x="300" y="340" width="210" height="70" as="geometry"/></mxCell><mxCell id="e1" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;endArrow=block" edge="1" parent="1" source="sensors" target="plc"><mxGeometry relative="1" as="geometry"/></mxCell><mxCell id="e2" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;endArrow=block" edge="1" parent="1" source="plc" target="act"><mxGeometry relative="1" as="geometry"/></mxCell><mxCell id="e3" style="edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;endArrow=block;startArrow=block" edge="1" parent="1" source="hmi" target="plc"><mxGeometry relative="1" as="geometry"/></mxCell></root></mxGraphModel></diagram></mxfile>"""
    path.write_text(xml, encoding="utf-8")


def generate_simulation_artifacts() -> pd.DataFrame:
    result = run_scenario(ScenarioConfig())
    frame = pd.DataFrame(result.records)
    frame.to_csv(ROOT / "data" / "normal_operation_log.csv", index=False)
    scan = frame[["time_s", "scan_period_s", "conveyor_motor", "motor_feedback", "state", "mode", "fault_id"]]
    scan.to_csv(ROOT / "data" / "scan_cycle_log.csv", index=False)
    render_dashboard(result.records, str(ROOT / "results" / "hmi_snapshot.png"))

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(frame.time_s, frame.speed_mps, color="#1f77b4", label="motor speed")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend()
    axes[1].step(frame.time_s, frame.conveyor_motor.astype(int), where="post", label="motor command")
    axes[1].step(frame.time_s, frame.diverter_extend.astype(int), where="post", label="diverter extend")
    axes[1].set_ylabel("Commands")
    axes[1].legend()
    axes[2].plot(frame.time_s, frame.product_count, color="#2ca02c", label="count")
    axes[2].set_ylabel("Products")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend()
    fig.suptitle("Normal Conveyor Scenario / Actual Simulation Log")
    fig.tight_layout()
    fig.savefig(ROOT / "results" / "control_response.png", dpi=160)
    plt.close(fig)
    return frame


def generate_pid_artifacts() -> pd.DataFrame:
    tunings = {
        "Manual": manual_tuning(),
        "Ziegler-Nichols relay": ziegler_nichols_tuning(MotorModel()),
        "Numerical grid search": optimize_tuning(),
    }
    rows = []
    simulations = {}
    for name, gains in tunings.items():
        simulation = simulate_controller(gains, measurement_noise_std=0.005)
        simulations[name] = simulation
        rows.append({"method": name, "kp": gains[0], "ki": gains[1], "kd": gains[2], **simulation["metrics"]})
    metrics = pd.DataFrame(rows)
    metrics.to_csv(ROOT / "data" / "pid_metrics.csv", index=False)
    response_rows = []
    for name, simulation in simulations.items():
        for index, time_s in enumerate(simulation["time_s"]):
            response_rows.append({"method": name, "time_s": time_s, "reference": simulation["reference"][index], "speed": simulation["speed"][index], "command": simulation["command"][index], "error": simulation["error"][index]})
    pd.DataFrame(response_rows).to_csv(ROOT / "data" / "pid_response_log.csv", index=False)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for name, simulation in simulations.items():
        axes[0].plot(simulation["time_s"], simulation["speed"], label=name)
        axes[1].plot(simulation["time_s"], simulation["command"], label=name)
    axes[0].plot(simulations["Manual"]["time_s"], simulations["Manual"]["reference"], "k--", label="reference")
    axes[0].set_ylabel("Speed (m/s)")
    axes[1].set_ylabel("Command (0..1)")
    axes[1].set_xlabel("Time (s)")
    axes[0].legend(frameon=False)
    axes[1].legend(frameon=False)
    axes[0].grid(alpha=0.25)
    axes[1].grid(alpha=0.25)
    fig.suptitle("PID Tuning Comparison / Actual Motor Model")
    fig.tight_layout()
    fig.savefig(ROOT / "results" / "pid_comparison.png", dpi=160)
    plt.close(fig)

    noisy = simulate_controller(manual_tuning(), measurement_noise_std=0.04)
    windup = simulate_controller(manual_tuning(), anti_windup_gain=0.0, duration_s=5.0)
    pd.DataFrame({"time_s": noisy["time_s"], "reference": noisy["reference"], "speed": noisy["speed"], "measurement": noisy["measurement"], "command": noisy["command"]}).to_csv(ROOT / "data" / "pid_noise_antiwindup_log.csv", index=False)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(noisy["time_s"], noisy["reference"], "k--", label="reference")
    axes[0].plot(noisy["time_s"], noisy["measurement"], alpha=0.35, label="noisy measurement")
    axes[0].plot(noisy["time_s"], noisy["speed"], label="filtered-control response")
    axes[1].plot(windup["time_s"], windup["command"], label="anti-windup enabled")
    axes[1].plot(noisy["time_s"], noisy["command"], label="noise + filtered derivative")
    axes[0].legend(frameon=False)
    axes[1].legend(frameon=False)
    axes[0].set_ylabel("Speed (m/s)")
    axes[1].set_ylabel("Command")
    axes[1].set_xlabel("Time (s)")
    fig.suptitle("Practical PID Effects / Saturation, Noise and Filtering")
    fig.tight_layout()
    fig.savefig(ROOT / "results" / "pid_practical_effects.png", dpi=160)
    plt.close(fig)
    return metrics


def generate_fault_artifacts() -> pd.DataFrame:
    scenarios = [
        ("F-002 Motor failure", ScenarioConfig(duration_s=3.0, arrivals=(), motor_failure=True)),
        ("F-003 Entry stuck ON", ScenarioConfig(duration_s=3.0, arrivals=(), sensor_faults={"entry_sensor": "stuck_on"})),
        ("F-004 Entry stuck OFF", ScenarioConfig(duration_s=12.0, arrivals=((1.0, False),), sensor_faults={"entry_sensor": "stuck_off"})),
        ("F-005 Diverter timeout", ScenarioConfig(duration_s=12.0, arrivals=((1.0, True),), diverter_failure=True)),
        ("F-006 Product timeout", ScenarioConfig(duration_s=22.0, arrivals=((1.0, False),), sensor_faults={"exit_sensor": "stuck_off"})),
        ("F-007 Impossible limits", ScenarioConfig(duration_s=2.0, arrivals=(), force_diverter_both_limits=True)),
    ]
    rows = []
    for name, config in scenarios:
        result = run_scenario(config)
        rows.append({"scenario": name, "fault_id": result.final_plc["fault_id"], "fault_message": result.final_plc["fault_message"], "alarm_seen": any(bool(row["alarm"]) for row in result.records), "safe_motor_at_end": not bool(result.records[-1]["conveyor_motor"])})
    frame = pd.DataFrame(rows)
    frame.to_csv(ROOT / "data" / "fault_test_results.csv", index=False)
    fig, ax = plt.subplots(figsize=(11, 5))
    colors = ["#2ca02c" if got.startswith(expected) else "#d62728" for got, expected in zip(frame.fault_id, ["F-002", "F-003", "F-004", "F-005", "F-006", "F-007"])]
    ax.bar(frame.scenario, [1] * len(frame), color=colors)
    ax.set_yticks([0, 1], ["FAIL", "PASS"])
    ax.set_title("Fault Injection Test Evidence / Actual PLC Results")
    ax.tick_params(axis="x", rotation=25)
    for index, row in frame.iterrows():
        ax.text(index, 0.5, row.fault_id, ha="center", va="center", color="white", weight="bold")
    fig.tight_layout()
    fig.savefig(ROOT / "results" / "fault_test_results.png", dpi=160)
    plt.close(fig)
    return frame


def generate_motor_sizing() -> pd.DataFrame:
    mass_conveyor_kg = 45.0
    product_mass_kg = 5.0
    speed_mps = 0.8
    radius_m = 0.05
    friction_coefficient = 0.04
    acceleration_mps2 = 0.4
    gravity = 9.81
    gearbox_efficiency = 0.90
    motor_speed_rpm = 1750.0
    total_mass = mass_conveyor_kg + product_mass_kg
    force_acceleration = total_mass * acceleration_mps2
    force_friction = friction_coefficient * total_mass * gravity
    required_force = force_acceleration + force_friction
    roller_torque = required_force * radius_m
    motor_torque_at_roller = roller_torque / gearbox_efficiency
    mechanical_power = required_force * speed_mps / gearbox_efficiency
    roller_angular_speed = speed_mps / radius_m
    motor_angular_speed = motor_speed_rpm * 2 * np.pi / 60
    gear_ratio = motor_angular_speed / roller_angular_speed
    data = [
        {"parameter": "conveyor_mass_kg", "value": mass_conveyor_kg, "unit": "kg", "basis": "design assumption"},
        {"parameter": "product_mass_kg", "value": product_mass_kg, "unit": "kg", "basis": "design assumption"},
        {"parameter": "belt_speed_mps", "value": speed_mps, "unit": "m/s", "basis": "design assumption"},
        {"parameter": "roller_radius_m", "value": radius_m, "unit": "m", "basis": "design assumption"},
        {"parameter": "friction_coefficient", "value": friction_coefficient, "unit": "-", "basis": "design assumption"},
        {"parameter": "acceleration_mps2", "value": acceleration_mps2, "unit": "m/s^2", "basis": "design assumption"},
        {"parameter": "required_acceleration_force_N", "value": force_acceleration, "unit": "N", "basis": "calculated"},
        {"parameter": "friction_force_N", "value": force_friction, "unit": "N", "basis": "calculated"},
        {"parameter": "required_force_N", "value": required_force, "unit": "N", "basis": "calculated"},
        {"parameter": "roller_torque_Nm", "value": roller_torque, "unit": "N*m", "basis": "calculated"},
        {"parameter": "motor_side_torque_Nm", "value": motor_torque_at_roller, "unit": "N*m", "basis": "calculated"},
        {"parameter": "mechanical_power_W", "value": mechanical_power, "unit": "W", "basis": "calculated"},
        {"parameter": "roller_speed_rad_s", "value": roller_angular_speed, "unit": "rad/s", "basis": "calculated"},
        {"parameter": "assumed_motor_speed_rpm", "value": motor_speed_rpm, "unit": "rpm", "basis": "design assumption"},
        {"parameter": "approximate_gear_ratio", "value": gear_ratio, "unit": "motor:roller", "basis": "calculated"},
    ]
    frame = pd.DataFrame(data)
    frame.to_csv(ROOT / "data" / "motor_sizing.csv", index=False)
    return frame


def generate_notebooks(repo_url: str = "https://github.com/SaadWajih99/industrial-conveyor-plc-hmi") -> None:
    bootstrap = f"""# Colab bootstrap: install only free packages and load this repository.\n!pip -q install numpy pandas matplotlib scipy\nfrom pathlib import Path\nimport subprocess, sys\nREPO_URL = {repo_url!r}\nPROJECT = Path('industrial-conveyor-plc-hmi')\nif not (PROJECT / 'simulation').exists():\n    subprocess.run(['git', 'clone', REPO_URL, str(PROJECT)], check=True)\nsys.path.insert(0, str(PROJECT.resolve()))\n"""
    write_notebook(ROOT / "analysis" / "01_system_simulation.ipynb", [
        ("markdown", "# 01 System Simulation\n\nRun the conveyor digital twin, inspect PLC state, counts, scan period and actual trends."),
        ("code", bootstrap),
        ("code", """import pandas as pd\nfrom simulation.testbench import ScenarioConfig, run_scenario\nresult = run_scenario(ScenarioConfig())\nlog = pd.DataFrame(result.records)\nprint(result.final_plc)\nprint(result.final_plant)\nlog.plot(x='time_s', y=['speed_mps', 'product_count'], subplots=True, figsize=(12, 6), grid=True)\n"""),
        ("markdown", "The baseline recipe spaces products so the single-product sequence can complete each transaction. In a production design, a queue or tracking structure would support tighter spacing."),
    ])
    write_notebook(ROOT / "analysis" / "02_pid_control_analysis.ipynb", [
        ("markdown", "# 02 PID Control Analysis\n\nCompare manual, relay-based Ziegler-Nichols and numerical grid-search tuning on the same motor model."),
        ("code", bootstrap),
        ("code", """import pandas as pd\nfrom control.pid_tuning import MotorModel, manual_tuning, ziegler_nichols_tuning, optimize_tuning, simulate_controller\ntunings = {'Manual': manual_tuning(), 'Ziegler-Nichols': ziegler_nichols_tuning(MotorModel()), 'Grid search': optimize_tuning()}\nrows = []\nfor method, gains in tunings.items():\n    result = simulate_controller(gains, measurement_noise_std=0.005)\n    rows.append({'method': method, **result['metrics']})\n    pd.Series(result['metrics']).plot.bar(title=method)\nprint(pd.DataFrame(rows).to_string(index=False))\n"""),
        ("markdown", "The model includes output saturation, conditional anti-windup/back-calculation and a first-order derivative filter. Metrics are calculated from the generated response arrays."),
    ])
    write_notebook(ROOT / "analysis" / "03_fault_testing.ipynb", [
        ("markdown", "# 03 Fault Testing\n\nInject faults into the same sensors and actuators used by the PLC simulation."),
        ("code", bootstrap),
        ("code", """from simulation.testbench import ScenarioConfig, run_scenario\nscenarios = {\n    'motor_failure': ScenarioConfig(duration_s=3, arrivals=(), motor_failure=True),\n    'entry_stuck_on': ScenarioConfig(duration_s=3, arrivals=(), sensor_faults={'entry_sensor': 'stuck_on'}),\n    'entry_stuck_off': ScenarioConfig(duration_s=12, arrivals=((1.0, False),), sensor_faults={'entry_sensor': 'stuck_off'}),\n    'diverter_timeout': ScenarioConfig(duration_s=12, arrivals=((1.0, True),), diverter_failure=True),\n    'product_timeout': ScenarioConfig(duration_s=22, arrivals=((1.0, False),), sensor_faults={'exit_sensor': 'stuck_off'}),\n    'impossible_limits': ScenarioConfig(duration_s=2, arrivals=(), force_diverter_both_limits=True),\n}\nfor name, config in scenarios.items():\n    result = run_scenario(config)\n    print(name, result.final_plc['fault_id'], result.final_plc['fault_message'], 'safe=', not result.records[-1]['conveyor_motor'])\n"""),
        ("markdown", "A physical installation would additionally validate wiring, diagnostic coverage and safety response independently of the standard PLC program."),
    ])
    write_notebook(ROOT / "analysis" / "04_motor_sizing.ipynb", [
        ("markdown", "# 04 Motor Sizing\n\nTransparent first-pass sizing calculation with every input labelled as an assumption."),
        ("code", """import math\nimport pandas as pd\nmass = 45.0 + 5.0\nv = 0.8\nr = 0.05\nmu = 0.04\na = 0.4\ng = 9.81\neta = 0.90\nforce = mass*a + mu*mass*g\ntorque = force*r/eta\npower = force*v/eta\nroller_omega = v/r\nmotor_rpm = 1750.0\nratio = (motor_rpm*2*math.pi/60)/roller_omega\nprint(pd.DataFrame([{'quantity':'force_N','value':force},{'quantity':'torque_Nm','value':torque},{'quantity':'power_W','value':power},{'quantity':'gear_ratio','value':ratio}]))\n"""),
        ("markdown", "This is not a vendor selection. Real sizing would include belt pretension, startup load, roller and gearbox losses, duty cycle, service factor, thermal limits, braking and safety requirements."),
    ])


def run_tests() -> tuple[int, str, int, int]:
    junit = ROOT / "data" / "pytest.xml"
    completed = subprocess.run([sys.executable, "-m", "pytest", "-q", f"--junitxml={junit}"], cwd=ROOT, text=True, capture_output=True)
    output = completed.stdout + "\n" + completed.stderr
    (ROOT / "data" / "pytest_output.txt").write_text(output, encoding="utf-8")
    tests = failures = errors = 0
    if junit.exists():
        root = ET.parse(junit).getroot()
        suites = [root] if root.tag == "testsuite" else list(root.findall("testsuite"))
        tests = sum(int(suite.attrib.get("tests", 0)) for suite in suites)
        failures = sum(int(suite.attrib.get("failures", 0)) for suite in suites)
        errors = sum(int(suite.attrib.get("errors", 0)) for suite in suites)
    return completed.returncode, output, tests, failures + errors


def generate_reports(test_return: int, test_output: str, test_count: int, test_failures: int, pid_metrics: pd.DataFrame, faults: pd.DataFrame, sizing: pd.DataFrame) -> None:
    expected_faults = ["F-002", "F-003", "F-004", "F-005", "F-006", "F-007"]
    fault_pass = all(str(actual).startswith(expected) for actual, expected in zip(faults.fault_id, expected_faults))
    required = [
        "README.md", "PROJECT_OVERVIEW.md", "CONTROL_NARRATIVE.md", "FUNCTIONAL_DESIGN_SPECIFICATION.md",
        "simulation/plant_model.py", "simulation/plc_simulation.py", "control/pid_controller.py", "hmi/hmi_simulation.py",
        "docs/io_list.csv", "docs/fault_matrix.csv", "docs/requirements_traceability_matrix.csv",
        "analysis/01_system_simulation.ipynb", "results/control_response.png", "results/pid_comparison.png", "results/fault_test_results.png",
    ]
    missing = [item for item in required if not (ROOT / item).exists()]
    status = "PASS" if test_return == 0 and test_count == 15 and test_failures == 0 and fault_pass and not missing else "FAIL"
    test_summary = f"""# Test Summary\n\n- Overall status: **{status}**\n- Test command: `python -m pytest -q`\n- Tests collected: {test_count}\n- Failures/errors: {test_failures}\n- Fault-injection matrix: {'PASS' if fault_pass else 'FAIL'}\n\n## Captured Output\n\n```text\n{test_output.strip()}\n```\n"""
    (ROOT / "results" / "test_summary.md").write_text(test_summary, encoding="utf-8")
    metrics_text = "\n".join(
        f"- {row.method}: rise {row.rise_time_s:.3f} s, settling {row.settling_time_s:.3f} s, overshoot {row.overshoot_percent:.2f} %, steady-state error {row.steady_state_error:.5f}, IAE {row.iae:.4f}, ISE {row.ise:.4f}"
        for row in pid_metrics.itertuples()
    )
    validation = f"""# Project Validation Report\n\nGenerated by `tools/generate_artifacts.py` from executable simulation and test outputs.\n\n## Implemented Features\n\n- PLC input/logic/output scan cycle with nominal 50 ms simulation period.\n- Automatic, manual, fault, emergency-stop and reset/recovery modes.\n- Motor seal-in, permissives, timers, edge counter, sort-request latch and diverter interlocks.\n- Sensor noise/failure injection, motor/diverter failure models and product timeout diagnostics.\n- HMI-style status, alarm and trend dashboard generated from the simulation log.\n- PID controller with output saturation, anti-windup and derivative filtering.\n- Manual, Ziegler-Nichols relay and numerical tuning comparisons.\n- First-pass motor/gearbox sizing calculation with labelled assumptions.\n- Architecture, I/O, state-machine, control-loop and wiring diagrams.\n\n## Tests Executed\n\n- Command: `python -m pytest -q`\n- Tests collected: {test_count}\n- Failures/errors: {test_failures}\n- Test status: **{'PASS' if test_return == 0 else 'FAIL'}**\n- Fault matrix status: **{'PASS' if fault_pass else 'FAIL'}**\n\n## PID Metrics\n\n{metrics_text}\n\n## Sizing Outputs\n\nThe full table is in `data/motor_sizing.csv`. It contains assumed conveyor mass, product mass, belt speed, radius, friction coefficient, acceleration and motor speed, followed by calculated force, torque, power and approximate ratio.\n\n## Known Limitations\n\n- This is a software model, not commissioning of a physical PLC or VFD.\n- The E-stop is represented as a software interlock and is not a certified safety function.\n- The conveyor sequence is intentionally single-product tracked; the baseline scenario spaces arrivals. A production system with close product spacing would need product tracking/queueing.\n- Sensor and actuator models are low-order behavioral models, not hardware characterization.\n- PID tuning is valid only for the stated first-order motor model and does not claim universal superiority.\n- HMI is a Python-generated dashboard, not a vendor-certified runtime.\n\n## Browser-Based Reproduction\n\nUse the four notebooks under `analysis/` in Google Colab. They install only free Python packages and clone the public repository.\n\n## Completeness Checks\n\n- Required-file check: **{'PASS' if not missing else 'FAIL'}**\n- Missing required files: {', '.join(missing) if missing else 'none'}\n- No API keys or passwords are used by the project.\n"""
    (ROOT / "PROJECT_VALIDATION_REPORT.md").write_text(validation, encoding="utf-8")


def main() -> None:
    generate_simulation_artifacts()
    pid_metrics = generate_pid_artifacts()
    faults = generate_fault_artifacts()
    sizing = generate_motor_sizing()
    architecture_diagram(ROOT / "diagrams" / "system_architecture.png")
    shutil.copy2(ROOT / "diagrams" / "system_architecture.png", ROOT / "docs" / "system_architecture.png")
    drawio_file(ROOT / "docs" / "system_architecture.drawio")
    control_loop_diagram(ROOT / "diagrams" / "control_loop.png")
    state_machine_diagram(ROOT / "diagrams" / "state_machine.png")
    wiring_diagram(ROOT / "diagrams" / "wiring_diagram.png")
    io_diagram(ROOT / "diagrams" / "io_diagram.png")
    generate_notebooks()
    return_code, output, count, failures = run_tests()
    generate_reports(return_code, output, count, failures, pid_metrics, faults, sizing)
    print(output.strip())
    print(f"Generated artifacts. tests={count} failures={failures} return_code={return_code}")
    if return_code:
        raise SystemExit(return_code)


if __name__ == "__main__":
    main()
