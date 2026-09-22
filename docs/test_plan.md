# Test Plan

| Test ID | Objective | Method | Expected result |
|---|---|---|---|
| TEST-001 | Normal automatic transport | Run default 35 s scenario | 3 products counted, 2 diverted, no fault |
| TEST-002 | Normal shutdown | Pulse Stop after startup | Motor output off and run latch clear |
| TEST-003 | Manual operation | Select Manual and command conveyor | Motor runs only while permissive is true |
| TEST-004 | Emergency stop | Open E-stop during running | Motion outputs off, alarm on, reset required |
| TEST-005 | Motor failure | Suppress motor response | F-002 latched |
| TEST-006 | Entry stuck ON | Force entry input high | F-003 latched |
| TEST-007 | Entry stuck OFF | Force entry input low through position | F-004 latched |
| TEST-008 | Diverter timeout | Suppress diverter response | F-005 latched |
| TEST-009 | Product transport timeout | Suppress exit detection | F-006 latched |
| TEST-010 | Impossible sensor state | Force both diverter limits | F-007 latched |
| TEST-011 | Fault reset | Clear condition and pulse Reset | Fault clears, outputs remain safe |
| TEST-012 | PID performance | Simulate three tuning methods | Finite metrics and bounded output |
