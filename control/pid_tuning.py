from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

import numpy as np

from .performance_metrics import calculate_metrics
from .pid_controller import PIDController


@dataclass(frozen=True)
class MotorModel:
    gain: float = 1.0
    time_constant_s: float = 0.55
    disturbance: float = 0.0

    def step(self, speed: float, command: float, dt_s: float) -> float:
        target = self.gain * command - self.disturbance
        return speed + (target - speed) * dt_s / self.time_constant_s


def manual_tuning() -> Tuple[float, float, float]:
    return 1.8, 2.2, 0.05


def relay_critical_parameters(model: MotorModel, dt_s: float = 0.01) -> Tuple[float, float]:
    """Estimate ultimate gain and period with a bounded relay experiment."""
    speed = 0.0
    output_high, output_low = 0.75, 0.25
    relay_state = True
    crossings = []
    values = []
    for index in range(int(30.0 / dt_s)):
        t = index * dt_s
        command = output_high if relay_state else output_low
        speed = model.step(speed, command, dt_s)
        # Hysteresis is essential; without it a noisy sample around the
        # switching threshold creates a meaningless high-frequency relay.
        should_switch = (relay_state and speed >= 0.55) or ((not relay_state) and speed <= 0.45)
        if should_switch:
            crossings.append(t)
            relay_state = not relay_state
        values.append(speed)
    if len(crossings) < 4:
        return 2.0, 1.0
    periods = np.diff(crossings[-5:]) * 2.0
    pu = float(np.mean(periods))
    amplitude = max((max(values[-int(5.0 / dt_s):]) - min(values[-int(5.0 / dt_s):])) / 2.0, 1e-6)
    relay_amplitude = (output_high - output_low) / 2.0
    ku = float(4.0 * relay_amplitude / (np.pi * amplitude))
    return ku, pu


def ziegler_nichols_tuning(model: MotorModel) -> Tuple[float, float, float]:
    ku, pu = relay_critical_parameters(model)
    return 0.6 * ku, 1.2 * ku / max(pu, 1e-6), 0.075 * ku * pu


def simulate_controller(
    gains: Tuple[float, float, float],
    duration_s: float = 8.0,
    dt_s: float = 0.01,
    setpoint_step_s: float = 0.5,
    measurement_noise_std: float = 0.0,
    anti_windup_gain: float = 2.0,
    derivative_filter_hz: float = 8.0,
    seed: int = 3,
) -> Dict[str, np.ndarray | Dict[str, float]]:
    rng = np.random.default_rng(seed)
    controller = PIDController(
        *gains,
        output_min=0.0,
        output_max=1.0,
        derivative_filter_hz=derivative_filter_hz,
        anti_windup_gain=anti_windup_gain,
    )
    model = MotorModel()
    time = np.arange(0.0, duration_s + dt_s / 2, dt_s)
    reference = np.where(time >= setpoint_step_s, 0.8, 0.0)
    speed = 0.0
    speeds, commands, errors, measured = [], [], [], []
    for setpoint in reference:
        measurement = speed + rng.normal(0.0, measurement_noise_std)
        command = controller.update(float(setpoint), float(measurement), dt_s)
        speed = model.step(speed, command, dt_s)
        measured.append(measurement)
        speeds.append(speed)
        commands.append(command)
        errors.append(float(setpoint - speed))
    metrics = calculate_metrics(time, reference, speeds)
    return {
        "time_s": time,
        "reference": reference,
        "speed": np.asarray(speeds),
        "measurement": np.asarray(measured),
        "command": np.asarray(commands),
        "error": np.asarray(errors),
        "metrics": metrics,
    }


def optimize_tuning(model: MotorModel | None = None) -> Tuple[float, float, float]:
    model = model or MotorModel()
    best = None
    kp_values = np.linspace(0.8, 3.6, 8)
    ki_values = np.linspace(0.8, 4.8, 9)
    kd_values = np.linspace(0.0, 0.12, 5)
    for kp in kp_values:
        for ki in ki_values:
            for kd in kd_values:
                result = simulate_controller((float(kp), float(ki), float(kd)), duration_s=5.0)
                metrics = result["metrics"]
                score = metrics["iae"] + 0.25 * metrics["ise"] + max(0.0, metrics["overshoot_percent"] - 5.0) * 0.05
                if best is None or score < best[0]:
                    best = (score, float(kp), float(ki), float(kd))
    assert best is not None
    return best[1], best[2], best[3]
