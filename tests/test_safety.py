from simulation.plc_simulation import ConveyorPLC, HMICommand, PLCInputs
from simulation.testbench import ScenarioConfig, run_scenario


def test_emergency_stop_drops_outputs_and_requires_reset():
    result = run_scenario(
        ScenarioConfig(
            duration_s=5.0,
            arrivals=(),
            emergency_stop_window=(1.5, 2.5),
            reset_time_s=3.0,
        )
    )
    active = [row for row in result.records if 1.5 <= row["time_s"] < 2.5]
    assert active
    assert all(row["conveyor_motor"] is False for row in active)
    assert all(row["alarm"] is True for row in active)
    assert result.final_plc["estop_latched"] is False
    assert result.final_plc["run_latched"] is False


def test_latched_fault_reset_returns_to_safe_off_state():
    plc = ConveyorPLC()
    plc.scan(
        PLCInputs(diverter_home=True, diverter_extended=True),
        HMICommand(mode="AUTO", start=True),
    )
    assert plc.status.fault_latched is True
    output = plc.scan(
        PLCInputs(diverter_home=True, diverter_extended=False, reset=True),
        HMICommand(mode="AUTO"),
    )
    assert plc.status.fault_latched is False
    assert output.conveyor_motor is False
