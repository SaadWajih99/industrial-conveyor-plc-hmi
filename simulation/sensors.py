from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Dict, Optional

from .plant_model import ConveyorPlant


@dataclass
class SensorBank:
    """Convert plant state to PLC inputs with optional injected failures."""

    noise_probability: float = 0.0
    seed: int = 7
    faults: Dict[str, str] = field(default_factory=dict)
    force_diverter_both_limits: bool = False
    _rng: random.Random = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def _apply_fault(self, name: str, value: bool) -> bool:
        fault = self.faults.get(name, "none")
        if fault == "stuck_on":
            return True
        if fault == "stuck_off":
            return False
        if self.noise_probability and self._rng.random() < self.noise_probability:
            return not value
        return value

    def read(self, plant: ConveyorPlant) -> Dict[str, bool]:
        entry = self._apply_fault("entry_sensor", plant.sensor_present("entry_sensor"))
        position = self._apply_fault("position_sensor", plant.sensor_present("position_sensor"))
        exit_sensor = self._apply_fault("exit_sensor", plant.sensor_present("exit_sensor"))
        home = self._apply_fault("diverter_home", plant.diverter_home)
        extended = self._apply_fault("diverter_extended", plant.diverter_extended)
        if self.force_diverter_both_limits:
            home = True
            extended = True

        # A real product-type or recipe signal would come from upstream
        # inspection. Here it is carried by the front product in the model.
        sort_request = any(
            not product.exited and product.position < plant.position_sensor_m
            and product.sort_requested
            for product in plant.products
        )
        return {
            "entry_sensor": entry,
            "position_sensor": position,
            "exit_sensor": exit_sensor,
            "motor_feedback": self._apply_fault("motor_feedback", plant.motor_feedback),
            "diverter_home": home,
            "diverter_extended": extended,
            "sort_request": sort_request,
        }
