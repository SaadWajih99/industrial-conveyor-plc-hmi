from __future__ import annotations

from dataclasses import dataclass

from .plant_model import ConveyorPlant


@dataclass(frozen=True)
class ActuatorCommand:
    conveyor_motor: bool = False
    diverter_extend: bool = False
    diverter_retract: bool = True
    green_run_lamp: bool = False
    red_fault_lamp: bool = False
    alarm: bool = False


class ActuatorInterface:
    """Map PLC output image to the simulated plant."""

    def apply(self, plant: ConveyorPlant, output: ActuatorCommand) -> None:
        plant.motor_command = output.conveyor_motor
        plant.diverter_extend_command = output.diverter_extend
        plant.diverter_retract_command = output.diverter_retract
