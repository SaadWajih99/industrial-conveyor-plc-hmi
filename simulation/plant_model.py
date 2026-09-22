from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class Product:
    """A product tracked by the transport model."""

    product_id: int
    sort_requested: bool
    position: float = 0.0
    diverted: bool = False
    exited: bool = False


@dataclass
class ConveyorPlant:
    """Low-order conveyor, motor and diverter model.

    The model is intentionally small enough to understand in an interview. It
    is not a finite-element mechanical model or a VFD firmware model.
    """

    length_m: float = 10.0
    entry_position_m: float = 0.2
    position_sensor_m: float = 5.8
    exit_position_m: float = 9.5
    diverter_position_m: float = 6.0
    max_speed_mps: float = 0.8
    motor_time_constant_s: float = 0.35
    diverter_time_constant_s: float = 0.20
    speed_mps: float = 0.0
    motor_command: bool = False
    motor_failure: bool = False
    diverter_extend_command: bool = False
    diverter_retract_command: bool = True
    diverter_failure: bool = False
    diverter_position: float = 0.0
    time_s: float = 0.0
    products: List[Product] = field(default_factory=list)
    next_product_id: int = 1
    exited_count: int = 0
    diverted_count: int = 0

    def add_product(self, sort_requested: bool = False) -> Product:
        product = Product(self.next_product_id, sort_requested)
        self.next_product_id += 1
        self.products.append(product)
        return product

    def step(self, dt_s: float) -> None:
        """Advance the plant one integration step."""

        target_speed = self.max_speed_mps if self.motor_command and not self.motor_failure else 0.0
        speed_delta = (target_speed - self.speed_mps) * dt_s / self.motor_time_constant_s
        self.speed_mps = max(0.0, min(self.max_speed_mps, self.speed_mps + speed_delta))

        if self.diverter_failure:
            target_diverter = 0.0
        elif self.diverter_extend_command and not self.diverter_retract_command:
            target_diverter = 1.0
        else:
            target_diverter = 0.0
        diverter_delta = (target_diverter - self.diverter_position) * dt_s / self.diverter_time_constant_s
        self.diverter_position = max(0.0, min(1.0, self.diverter_position + diverter_delta))

        for product in self.products:
            if product.exited:
                continue
            product.position += self.speed_mps * dt_s
            if (
                product.sort_requested
                and not product.diverted
                and self.diverter_position > 0.75
                and abs(product.position - self.diverter_position_m) < 0.30
            ):
                product.diverted = True
                self.diverted_count += 1
            if product.position >= self.exit_position_m:
                product.exited = True
                self.exited_count += 1

        self.products = [p for p in self.products if p.position < self.length_m + 0.5]
        self.time_s += dt_s

    def sensor_present(self, name: str) -> bool:
        """Return ideal sensor state for a named digital input."""

        if name == "entry_sensor":
            center, width = self.entry_position_m, 0.18
        elif name == "position_sensor":
            center, width = self.position_sensor_m, 0.22
        elif name == "exit_sensor":
            center, width = self.exit_position_m, 0.18
        else:
            raise KeyError(f"Unknown product sensor: {name}")
        return any(not p.exited and abs(p.position - center) <= width for p in self.products)

    @property
    def motor_feedback(self) -> bool:
        return self.speed_mps >= 0.25 * self.max_speed_mps

    @property
    def diverter_home(self) -> bool:
        return self.diverter_position <= 0.05

    @property
    def diverter_extended(self) -> bool:
        return self.diverter_position >= 0.95
