# PLC Logic Design

## Scan Model

Every 50 ms the simulation performs:

1. Read the sensor and pushbutton image into `PLCInputs`.
2. Evaluate E-stop, reset, fault, mode, latches, timers, sequence and interlocks.
3. Build one `ActuatorCommand` output image.
4. Apply outputs to the plant and advance the physical model.

The next scan sees the plant response and motor/diverter feedback. This makes input-to-output command latency one scan in the software model, with a configured nominal period of 50 ms. The real sensor-to-energy-removal response of a safety circuit is outside this software simulation.

## Ladder-Oriented Rungs

The executable equivalent is in `simulation/plc_simulation.py`.

```text
Rung 1: E_STOP_OK AND NOT FAULT_LATCHED AND NOT ESTOP_LATCHED -> PERMISSIVE

Rung 2: AUTO_MODE AND START AND PERMISSIVE -> RUN_LATCH
        RUN_LATCH seals around START
        STOP OR NOT PERMISSIVE -> RUN_LATCH reset

Rung 3: PERMISSIVE AND ((AUTO_MODE AND RUN_LATCH) OR
        (MANUAL_MODE AND MANUAL_CONVEYOR)) -> Q0.0 CONVEYOR_MOTOR

Rung 4: AUTO_MODE AND STATE_SORTING AND PERMISSIVE -> Q0.1 DIVERTER_EXTEND
        NOT Q0.1 -> Q0.2 DIVERTER_RETRACT

Rung 5: Q0.0 AND NOT MOTOR_FEEDBACK TON T_MOTOR_FB
        T_MOTOR_FB.DN -> latch F-002

Rung 6: Q0.1 AND NOT DIVERTER_EXTENDED TON T_DIV_EXT
        T_DIV_EXT.DN -> latch F-005

Rung 7: ENTRY_SENSOR AND NOT ENTRY_SENSOR_PREV -> PRODUCT_ACTIVE,
        latch SORT_REQUEST, reset T_PRODUCT
        PRODUCT_ACTIVE AND NOT EXIT_RISING TON T_PRODUCT
        T_PRODUCT.DN -> latch F-006

Rung 8: EXIT_SENSOR AND NOT EXIT_SENSOR_PREV -> increment PRODUCT_COUNT,
        clear PRODUCT_ACTIVE and SORT_REQUEST

Rung 9: FAULT_LATCHED AND RESET AND E_STOP_OK AND DIVERTER_HOME -> clear fault
        E_STOP event always clears RUN_LATCH and requires reset before restart
```

## Timer and Counter Semantics

Timers accumulate `dt_s` once their enabling condition is true. Counters increment only on a rising edge, not every scan while a sensor remains active. This prevents a wide photoelectric beam from creating multiple product counts.

## Safety Boundary

The E-stop interlock is intentionally represented in both the software logic and output safe state, but it is not a certified safety function. A real design would use a safety-rated circuit, monitored contactors or STO, and independent validation.
