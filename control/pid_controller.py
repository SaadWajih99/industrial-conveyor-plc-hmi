from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDController:
    """PID controller with output limiting, conditional anti-windup and D filtering."""

    kp: float
    ki: float
    kd: float
    output_min: float = 0.0
    output_max: float = 1.0
    derivative_filter_hz: float = 8.0
    anti_windup_gain: float = 2.0
    integral: float = 0.0
    previous_measurement: float = 0.0
    filtered_derivative: float = 0.0
    initialized: bool = False

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_measurement = 0.0
        self.filtered_derivative = 0.0
        self.initialized = False

    def update(self, setpoint: float, measurement: float, dt_s: float) -> float:
        error = setpoint - measurement
        if not self.initialized:
            self.previous_measurement = measurement
            self.initialized = True
        raw_derivative = -(measurement - self.previous_measurement) / max(dt_s, 1e-9)
        alpha = (2.0 * 3.141592653589793 * self.derivative_filter_hz * dt_s) / (
            1.0 + 2.0 * 3.141592653589793 * self.derivative_filter_hz * dt_s
        )
        self.filtered_derivative += alpha * (raw_derivative - self.filtered_derivative)

        unsaturated = self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative
        output = max(self.output_min, min(self.output_max, unsaturated))

        # Back-calculation prevents the integral from growing while the output
        # is saturated, while still allowing integration when it helps recover.
        saturation_error = output - unsaturated
        self.integral += (error + self.anti_windup_gain * saturation_error) * dt_s
        self.previous_measurement = measurement
        return output
