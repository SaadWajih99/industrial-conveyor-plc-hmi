# Functional Design Specification

## 1. Scope

This FDS defines a browser-reproducible software model of a conveyor, motor, diverter, sensors, PLC sequence, HMI view and verification suite.

## 2. Functional Requirements

- Read field inputs into a PLC input image every scan.
- Apply E-stop, mode, start/stop, permissive and fault logic before enabling outputs.
- Run automatic product transport and sorting.
- Permit constrained manual motion.
- Detect motor, sensor, diverter, timeout and impossible-state faults.
- Count products at the exit rising edge.
- Expose operator status, alarms and trends.
- Provide a separate closed-loop speed controller study with measurable performance metrics.

## 3. I/O and Signal Philosophy

The complete mapping is in `docs/io_list.csv`. The E-stop is modelled as an NC-style healthy signal (`I0.0 = 1` means the circuit is closed and healthy). Outputs are command-level signals; a real installation would include output drivers, contactors/VFD interfaces, feedback wiring and safety circuits.

## 4. Control Philosophy

The PLC is organized as a scan-cycle function: read inputs, execute logic, update outputs. Timers use scan-period accumulation, counters use edge detection, and faults latch until reset conditions are satisfied. Automatic sequencing is explicit in `simulation/state_machine.py`.

## 5. Alarms and Interlocks

Alarm definitions and recovery are in `docs/fault_matrix.csv`. Critical interlocks include E-stop healthy, no latched fault, motor feedback during startup, mutually exclusive diverter commands, diverter home/extended plausibility and product transport timeout.

## 6. HMI

The HMI model reports PLC status, current mode, sequence state, motor command and feedback, counts, scan time, faults and trends. The generated PNG is evidence from the same scenario log used in the tests.

## 7. Testing

`pytest -q` runs normal operation, mode, safety, fault and control tests. `tools/run_project.py` also generates a machine-readable test summary, traceability results and validation report.

## 8. Physical Implementation Boundary

This software model does not implement a safety-rated function, electrical isolation, certified PLC runtime, VFD firmware, real-time operating system or mechanical guarding. Physical design would require hazard analysis, risk reduction, safety validation and site commissioning by qualified personnel.
