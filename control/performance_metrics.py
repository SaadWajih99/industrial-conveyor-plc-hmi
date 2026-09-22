from __future__ import annotations

from typing import Dict, Iterable

import numpy as np


def calculate_metrics(time_s: Iterable[float], reference: Iterable[float], response: Iterable[float]) -> Dict[str, float]:
    t = np.asarray(list(time_s), dtype=float)
    r = np.asarray(list(reference), dtype=float)
    y = np.asarray(list(response), dtype=float)
    if len(t) < 2:
        raise ValueError("At least two samples are required")
    final_value = float(r[-1])
    step_size = max(abs(final_value - r[0]), 1e-12)
    low = r[0] + 0.1 * step_size
    high = r[0] + 0.9 * step_size
    above_low = np.flatnonzero(y >= low)
    above_high = np.flatnonzero(y >= high)
    rise_time = float("nan")
    if len(above_low) and len(above_high):
        rise_time = float(t[above_high[0]] - t[above_low[0]])
    band = 0.02 * step_size
    settling_time = float("nan")
    for i in range(len(y)):
        if np.all(np.abs(y[i:] - final_value) <= band):
            settling_time = float(t[i])
            break
    overshoot = max(0.0, float((np.max(y) - final_value) / step_size * 100.0))
    error = r - y
    integrate = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    return {
        "rise_time_s": rise_time,
        "settling_time_s": settling_time,
        "overshoot_percent": overshoot,
        "steady_state_error": float(abs(error[-1])),
        "iae": float(integrate(np.abs(error), t)),
        "ise": float(integrate(error**2, t)),
    }
