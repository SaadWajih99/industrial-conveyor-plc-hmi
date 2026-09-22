from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .actuators import ActuatorCommand
from .state_machine import AutoState, Mode


@dataclass(frozen=True)
class PLCInputs:
    emergency_stop_ok: bool = True
    start: bool = False
    stop: bool = False
    reset: bool = False
    entry_sensor: bool = False
    position_sensor: bool = False
    exit_sensor: bool = False
    motor_feedback: bool = False
    diverter_home: bool = True
    diverter_extended: bool = False
    sort_request: bool = False


@dataclass(frozen=True)
class HMICommand:
    mode: str = "OFF"
    start: bool = False
    stop: bool = False
    reset: bool = False
    manual_conveyor: bool = False
    manual_diverter_extend: bool = False
    manual_diverter_retract: bool = False


@dataclass
class PLCConfig:
    scan_period_s: float = 0.05
    motor_feedback_timeout_s: float = 0.50
    diverter_timeout_s: float = 0.80
    product_timeout_s: float = 18.0
    entry_stuck_on_timeout_s: float = 2.0


@dataclass
class PLCStatus:
    mode: str = Mode.OFF.value
    state: str = AutoState.IDLE.value
    run_latched: bool = False
    fault_latched: bool = False
    estop_latched: bool = False
    fault_id: str = ""
    fault_message: str = ""
    product_count: int = 0
    cycle_count: int = 0
    scan_count: int = 0
    last_scan_period_s: float = 0.05


class ConveyorPLC:
    """PLC-style logic with explicit timers, latches and output image."""

    def __init__(self, config: Optional[PLCConfig] = None) -> None:
        self.config = config or PLCConfig()
        self.status = PLCStatus(last_scan_period_s=self.config.scan_period_s)
        self.fault_history: List[str] = []
        self._previous_inputs = PLCInputs()
        self._previous_outputs = ActuatorCommand()
        self._motor_feedback_timer = 0.0
        self._diverter_timer = 0.0
        self._product_timer = 0.0
        self._entry_active_timer = 0.0
        self._entry_seen_current_product = False
        self._product_active = False
        self._sort_latched = False

    def _latch_fault(self, fault_id: str, message: str) -> None:
        if not self.status.fault_latched:
            self.fault_history.append(fault_id)
        self.status.fault_latched = True
        self.status.fault_id = fault_id
        self.status.fault_message = message
        self.status.mode = Mode.FAULT.value
        self.status.state = AutoState.FAULT.value
        self.status.run_latched = False

    def _physical_fault(self, inputs: PLCInputs, dt_s: float) -> Optional[tuple[str, str]]:
        if inputs.diverter_home and inputs.diverter_extended:
            return "F-007", "Impossible diverter limit combination"

        if self._previous_outputs.conveyor_motor and not inputs.motor_feedback:
            self._motor_feedback_timer += dt_s
        else:
            self._motor_feedback_timer = 0.0
        if self._motor_feedback_timer >= self.config.motor_feedback_timeout_s:
            return "F-002", "Motor command active without feedback"

        if self._previous_outputs.diverter_extend and not inputs.diverter_extended:
            self._diverter_timer += dt_s
        else:
            self._diverter_timer = 0.0
        if self._diverter_timer >= self.config.diverter_timeout_s:
            return "F-005", "Diverter extension timeout"

        if inputs.entry_sensor:
            self._entry_active_timer += dt_s
        else:
            self._entry_active_timer = 0.0
        if self._entry_active_timer >= self.config.entry_stuck_on_timeout_s:
            return "F-003", "Entry sensor stuck ON"

        if inputs.position_sensor and not self._entry_seen_current_product:
            return "F-004", "Product reached position without entry detection"

        if self._product_active:
            self._product_timer += dt_s
            if self._product_timer >= self.config.product_timeout_s:
                return "F-006", "Product transport timeout"
        return None

    def _advance_sequence(self, inputs: PLCInputs, hmi: HMICommand) -> None:
        if self.status.mode != Mode.AUTOMATIC.value:
            self.status.state = AutoState.IDLE.value
            return
        if not self.status.run_latched or hmi.stop:
            self.status.state = AutoState.IDLE.value
            return
        state = AutoState(self.status.state)
        entry_rising = inputs.entry_sensor and not self._previous_inputs.entry_sensor
        exit_rising = inputs.exit_sensor and not self._previous_inputs.exit_sensor

        if state == AutoState.IDLE:
            self.status.state = AutoState.STARTUP.value
        elif state == AutoState.STARTUP:
            if inputs.motor_feedback:
                self.status.state = AutoState.CONVEYING.value
        elif state == AutoState.CONVEYING:
            if entry_rising or inputs.entry_sensor:
                self._product_active = True
                self._product_timer = 0.0
                self._entry_seen_current_product = True
                self._sort_latched = inputs.sort_request
                self.status.state = AutoState.PRODUCT_DETECTED.value
        elif state == AutoState.PRODUCT_DETECTED:
            if inputs.position_sensor:
                self.status.state = AutoState.POSITIONING.value
        elif state == AutoState.POSITIONING:
            if self._sort_latched and inputs.position_sensor:
                self.status.state = AutoState.SORTING.value
            elif exit_rising:
                self.status.state = AutoState.PRODUCT_EXIT.value
        elif state == AutoState.SORTING:
            if exit_rising:
                self.status.state = AutoState.PRODUCT_EXIT.value
        elif state == AutoState.PRODUCT_EXIT:
            if not inputs.exit_sensor:
                self.status.state = AutoState.CONVEYING.value

    def scan(self, inputs: PLCInputs, hmi: HMICommand, dt_s: Optional[float] = None) -> ActuatorCommand:
        """Execute one read-input, logic, update-output PLC scan."""

        dt = dt_s or self.config.scan_period_s
        self.status.scan_count += 1
        self.status.last_scan_period_s = dt

        if inputs.exit_sensor and not self._previous_inputs.exit_sensor:
            self.status.product_count += 1
            self.status.cycle_count += 1
            self._product_active = False
            self._product_timer = 0.0
            self._entry_seen_current_product = False
            self._sort_latched = False

        if not inputs.emergency_stop_ok:
            self.status.estop_latched = True
            self.status.mode = Mode.EMERGENCY_STOP.value
            self.status.state = AutoState.EMERGENCY_STOP.value
            self.status.run_latched = False
        elif self.status.estop_latched:
            if inputs.reset:
                self.status.estop_latched = False
                self.status.mode = Mode.RESET.value
                self.status.state = AutoState.RESET.value
        elif self.status.fault_latched:
            if inputs.reset and inputs.diverter_home and not inputs.diverter_extended:
                self.status.fault_latched = False
                self.status.fault_id = ""
                self.status.fault_message = ""
                self.status.mode = Mode.RESET.value
                self.status.state = AutoState.RESET.value
            else:
                self.status.mode = Mode.FAULT.value
                self.status.state = AutoState.FAULT.value
        else:
            requested_mode = hmi.mode.upper()
            if requested_mode == "AUTO":
                self.status.mode = Mode.AUTOMATIC.value
            elif requested_mode in {"MANUAL", "OFF"}:
                self.status.mode = requested_mode
            if hmi.start and self.status.mode == Mode.AUTOMATIC.value:
                self.status.run_latched = True
            if hmi.stop or self.status.mode == Mode.OFF.value:
                self.status.run_latched = False
            physical_fault = self._physical_fault(inputs, dt)
            if physical_fault:
                self._latch_fault(*physical_fault)

        if self.status.mode == Mode.RESET.value:
            self.status.mode = Mode.OFF.value
            self.status.state = AutoState.IDLE.value
            self.status.run_latched = False

        if not self.status.fault_latched and not self.status.estop_latched:
            self._advance_sequence(inputs, hmi)

        permissive = (
            inputs.emergency_stop_ok
            and not self.status.fault_latched
            and not self.status.estop_latched
            and not (inputs.diverter_home and inputs.diverter_extended)
        )
        auto_motor = self.status.mode == Mode.AUTOMATIC.value and self.status.run_latched
        manual_motor = self.status.mode == Mode.MANUAL.value and hmi.manual_conveyor
        motor = permissive and (auto_motor or manual_motor)

        auto_extend = self.status.mode == Mode.AUTOMATIC.value and self.status.state == AutoState.SORTING.value
        manual_extend = self.status.mode == Mode.MANUAL.value and hmi.manual_diverter_extend
        manual_retract = self.status.mode == Mode.MANUAL.value and hmi.manual_diverter_retract
        diverter_extend = permissive and (auto_extend or manual_extend) and not manual_retract
        diverter_retract = not diverter_extend

        output = ActuatorCommand(
            conveyor_motor=motor,
            diverter_extend=diverter_extend,
            diverter_retract=diverter_retract,
            green_run_lamp=motor,
            red_fault_lamp=self.status.fault_latched or self.status.estop_latched,
            alarm=self.status.fault_latched or self.status.estop_latched,
        )
        self._previous_inputs = inputs
        self._previous_outputs = output
        return output

    def snapshot(self) -> Dict[str, object]:
        return {
            "mode": self.status.mode,
            "state": self.status.state,
            "run_latched": self.status.run_latched,
            "fault_latched": self.status.fault_latched,
            "estop_latched": self.status.estop_latched,
            "fault_id": self.status.fault_id,
            "fault_message": self.status.fault_message,
            "product_count": self.status.product_count,
            "cycle_count": self.status.cycle_count,
            "scan_count": self.status.scan_count,
            "scan_period_s": self.status.last_scan_period_s,
        }
