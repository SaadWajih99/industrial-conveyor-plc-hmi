from __future__ import annotations

from enum import Enum


class Mode(str, Enum):
    OFF = "OFF"
    MANUAL = "MANUAL"
    AUTOMATIC = "AUTOMATIC"
    FAULT = "FAULT"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RESET = "RESET"


class AutoState(str, Enum):
    IDLE = "IDLE"
    STARTUP = "STARTUP"
    CONVEYING = "CONVEYING"
    PRODUCT_DETECTED = "PRODUCT_DETECTED"
    POSITIONING = "POSITIONING"
    SORTING = "SORTING"
    PRODUCT_EXIT = "PRODUCT_EXIT"
    FAULT = "FAULT"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RESET = "RESET"


SEQUENCE_TABLE = [
    ("IDLE", "Automatic run latch", "Transition to STARTUP"),
    ("STARTUP", "Motor feedback healthy", "Transition to CONVEYING"),
    ("CONVEYING", "Entry sensor rising edge", "Latch product and transition to PRODUCT_DETECTED"),
    ("PRODUCT_DETECTED", "Position sensor active", "Transition to POSITIONING"),
    ("POSITIONING", "Sort request latched", "Transition to SORTING"),
    ("SORTING", "Exit sensor rising edge", "Transition to PRODUCT_EXIT"),
    ("PRODUCT_EXIT", "Exit counted", "Clear product latch and return to CONVEYING"),
]
