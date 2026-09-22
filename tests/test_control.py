import numpy as np

from control.pid_tuning import (
    MotorModel,
    manual_tuning,
    optimize_tuning,
    simulate_controller,
    ziegler_nichols_tuning,
)


def test_manual_pid_reaches_reference_with_low_error():
    result = simulate_controller(manual_tuning(), measurement_noise_std=0.005)
    metrics = result["metrics"]
    assert metrics["steady_state_error"] < 0.03
    assert np.isfinite(metrics["iae"])
    assert np.isfinite(metrics["ise"])


def test_three_tuning_methods_produce_executable_results():
    gains = [manual_tuning(), ziegler_nichols_tuning(MotorModel()), optimize_tuning()]
    for tuning in gains:
        result = simulate_controller(tuning)
        assert np.isfinite(result["speed"]).all()
        assert result["metrics"]["steady_state_error"] < 0.08


def test_pid_output_is_saturated_and_noise_filtered():
    result = simulate_controller(manual_tuning(), measurement_noise_std=0.04)
    assert float(np.max(result["command"])) <= 1.0 + 1e-12
    assert float(np.min(result["command"])) >= -1e-12
