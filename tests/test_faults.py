from simulation.testbench import ScenarioConfig, run_scenario


def test_motor_failure_is_detected():
    result = run_scenario(ScenarioConfig(duration_s=3.0, arrivals=(), motor_failure=True))
    assert result.final_plc["fault_id"] == "F-002"


def test_entry_sensor_stuck_on_is_detected():
    result = run_scenario(
        ScenarioConfig(duration_s=3.0, arrivals=(), sensor_faults={"entry_sensor": "stuck_on"})
    )
    assert result.final_plc["fault_id"] == "F-003"


def test_entry_sensor_stuck_off_is_detected_at_position():
    result = run_scenario(
        ScenarioConfig(
            duration_s=12.0,
            arrivals=((1.0, False),),
            sensor_faults={"entry_sensor": "stuck_off"},
        )
    )
    assert result.final_plc["fault_id"] == "F-004"


def test_diverter_timeout_is_detected():
    result = run_scenario(
        ScenarioConfig(duration_s=12.0, arrivals=((1.0, True),), diverter_failure=True)
    )
    assert result.final_plc["fault_id"] == "F-005"


def test_product_transport_timeout_is_detected():
    result = run_scenario(
        ScenarioConfig(
            duration_s=22.0,
            arrivals=((1.0, False),),
            sensor_faults={"exit_sensor": "stuck_off"},
        )
    )
    assert result.final_plc["fault_id"] == "F-006"


def test_impossible_sensor_combination_is_detected():
    result = run_scenario(ScenarioConfig(duration_s=2.0, arrivals=(), force_diverter_both_limits=True))
    assert result.final_plc["fault_id"] == "F-007"
