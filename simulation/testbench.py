from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .actuators import ActuatorInterface
from .plant_model import ConveyorPlant
from .plc_simulation import ConveyorPLC, HMICommand, PLCInputs
from .sensors import SensorBank


@dataclass
class ScenarioConfig:
    duration_s: float = 45.0
    dt_s: float = 0.05
    # The baseline recipe leaves enough spacing for the single-product
    # sequencer to complete one transaction before the next entry pulse.
    arrivals: Sequence[Tuple[float, bool]] = ((1.0, True), (15.0, False), (29.0, True))
    mode: str = "AUTO"
    start_time_s: Optional[float] = 0.2
    stop_time_s: Optional[float] = None
    reset_time_s: Optional[float] = None
    emergency_stop_window: Optional[Tuple[float, float]] = None
    noise_probability: float = 0.0
    sensor_faults: Dict[str, str] = field(default_factory=dict)
    motor_failure: bool = False
    diverter_failure: bool = False
    force_diverter_both_limits: bool = False
    manual_conveyor: bool = False
    manual_diverter_extend: bool = False
    manual_diverter_retract: bool = False


@dataclass
class SimulationResult:
    records: List[Dict[str, object]]
    final_plc: Dict[str, object]
    final_plant: Dict[str, object]

    def rows(self) -> List[Dict[str, object]]:
        return self.records


def run_scenario(config: Optional[ScenarioConfig] = None) -> SimulationResult:
    cfg = config or ScenarioConfig()
    plant = ConveyorPlant()
    plant.motor_failure = cfg.motor_failure
    plant.diverter_failure = cfg.diverter_failure
    sensors = SensorBank(
        noise_probability=cfg.noise_probability,
        faults=dict(cfg.sensor_faults),
        force_diverter_both_limits=cfg.force_diverter_both_limits,
    )
    plc = ConveyorPLC()
    actuators = ActuatorInterface()
    records: List[Dict[str, object]] = []
    arrival_index = 0
    steps = int(round(cfg.duration_s / cfg.dt_s))

    for step in range(steps):
        t = step * cfg.dt_s
        while arrival_index < len(cfg.arrivals) and cfg.arrivals[arrival_index][0] <= t + 1e-9:
            _, sort_requested = cfg.arrivals[arrival_index]
            plant.add_product(sort_requested=sort_requested)
            arrival_index += 1

        estop_active = (
            cfg.emergency_stop_window is not None
            and cfg.emergency_stop_window[0] <= t < cfg.emergency_stop_window[1]
        )
        pulse = lambda event_time: event_time is not None and abs(t - event_time) < cfg.dt_s / 2
        hmi = HMICommand(
            mode=cfg.mode,
            start=pulse(cfg.start_time_s),
            stop=pulse(cfg.stop_time_s),
            reset=pulse(cfg.reset_time_s),
            manual_conveyor=cfg.manual_conveyor,
            manual_diverter_extend=cfg.manual_diverter_extend,
            manual_diverter_retract=cfg.manual_diverter_retract,
        )
        raw_inputs = sensors.read(plant)
        inputs = PLCInputs(
            emergency_stop_ok=not estop_active,
            start=hmi.start,
            stop=hmi.stop,
            reset=hmi.reset,
            **raw_inputs,
        )
        output = plc.scan(inputs, hmi, cfg.dt_s)
        actuators.apply(plant, output)
        plant.step(cfg.dt_s)
        row = {
            "time_s": round(t, 6),
            "speed_mps": plant.speed_mps,
            "diverter_position": plant.diverter_position,
            "entry_sensor": inputs.entry_sensor,
            "position_sensor": inputs.position_sensor,
            "exit_sensor": inputs.exit_sensor,
            "motor_feedback": inputs.motor_feedback,
            "sort_request": inputs.sort_request,
            "conveyor_motor": output.conveyor_motor,
            "diverter_extend": output.diverter_extend,
            "diverter_retract": output.diverter_retract,
            "alarm": output.alarm,
            **plc.snapshot(),
        }
        records.append(row)

    return SimulationResult(
        records=records,
        final_plc=plc.snapshot(),
        final_plant={
            "time_s": plant.time_s,
            "speed_mps": plant.speed_mps,
            "exited_count": plant.exited_count,
            "diverted_count": plant.diverted_count,
            "diverter_home": plant.diverter_home,
            "diverter_extended": plant.diverter_extended,
        },
    )
