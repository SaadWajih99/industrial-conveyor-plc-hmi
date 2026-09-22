from simulation.plc_simulation import ConveyorPLC, HMICommand, PLCInputs
from simulation.testbench import ScenarioConfig, run_scenario


def test_normal_automatic_operation_counts_and_diverts_products():
    result = run_scenario(ScenarioConfig())
    assert result.final_plc["fault_latched"] is False
    assert result.final_plc["product_count"] == 3
    assert result.final_plant["exited_count"] == 3
    assert result.final_plant["diverted_count"] == 2


def test_normal_shutdown_removes_motor_command():
    result = run_scenario(ScenarioConfig(duration_s=5.0, stop_time_s=3.0))
    assert result.final_plc["run_latched"] is False
    assert result.records[-1]["conveyor_motor"] is False


def test_manual_mode_keeps_safety_interlock_and_runs_when_commanded():
    result = run_scenario(
        ScenarioConfig(
            duration_s=2.0,
            mode="MANUAL",
            start_time_s=None,
            manual_conveyor=True,
        )
    )
    assert result.final_plc["mode"] == "MANUAL"
    assert result.records[-1]["conveyor_motor"] is True


def test_mode_change_does_not_bypass_safety():
    plc = ConveyorPLC()
    plc.scan(PLCInputs(emergency_stop_ok=False), HMICommand(mode="AUTO", start=True))
    output = plc.scan(
        PLCInputs(emergency_stop_ok=False),
        HMICommand(mode="MANUAL", manual_conveyor=True),
    )
    assert output.conveyor_motor is False
    assert output.diverter_extend is False
