# Project Overview

## Purpose

The project models a 10 m conveyor cell that transports products from an entry photoelectric sensor to an exit sensor. A product-classification bit requests diversion at a mid-line diverter. The PLC simulation supervises the cell, while a simplified motor model provides feedback for a closed-loop speed-control study.

## Design Basis

| Item | Value | Status |
|---|---:|---|
| Conveyor length | 10.0 m | Design assumption |
| Nominal belt speed | 0.8 m/s | Design assumption |
| PLC scan period | 50 ms | Simulation parameter |
| Entry sensor | 0.2 m | Design assumption |
| Position/diverter sensor | 5.8 m | Design assumption |
| Exit sensor | 9.5 m | Design assumption |
| Product timeout | 18 s | PLC parameter |
| Diverter extension timeout | 0.8 s | PLC parameter |

## Separation of Concerns

- `simulation/plant_model.py` represents the physical process.
- `simulation/sensors.py` turns plant state into noisy digital inputs.
- `simulation/plc_simulation.py` implements the PLC input/logic/output scan.
- `simulation/state_machine.py` defines the automatic sequence states.
- `simulation/actuators.py` maps output commands to plant commands.
- `control/` contains the continuous motor-speed controller study.
- `hmi/` formats actual PLC and plant logs into operator-facing status and trends.
- `tests/` verifies safety, sequencing, faults and controller behavior.

## Interview Boundary

The most important engineering distinction is that the simulation demonstrates behavior and verification logic, not a certified machine. A physical implementation would add a safety-rated E-stop circuit, safety relay or safety PLC, STO-capable drive, 24 VDC electrical protection, physical I/O validation, VFD parameterization, mechanical guarding and commissioning procedures.
