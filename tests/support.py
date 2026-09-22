from __future__ import annotations

from typing import Dict, Iterable

from simulation.testbench import ScenarioConfig, SimulationResult, run_scenario


def fault_for(result: SimulationResult) -> str:
    return str(result.final_plc.get("fault_id", ""))


def rows_with_alarm(result: SimulationResult) -> Iterable[Dict[str, object]]:
    return (row for row in result.records if bool(row.get("alarm")))
