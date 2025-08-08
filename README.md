# Arduino Robot Control

This repository contains a single `main.c` sketch that drives a small robot using an Arduino-compatible board. The sketch demonstrates:

- **Two stepper motors** driven by DRV8825 drivers (step and direction lines only)
- **Two DC motors** run for fixed durations (no encoders)
- **Two solenoids** via single-pin control
- **Three push buttons** (START, RESET, STOP) configured as interrupts triggered on release (rising edge)
- A `switch`/`enum` state machine (`IDLE`, `RESET`, `SEQUENCE`, `DONE`)

## Building

Compile and upload the sketch with your preferred Arduino toolchain. For example, using `arduino-cli`:

```bash
arduino-cli compile --fqbn <board> .
arduino-cli upload --port <port> --fqbn <board> .
```

The code relies only on the Arduino core and runs the sequence inside the `loop()` function.

Debug messages use `Serial.print` and appear on the hardware serial port at 115200 baud. Set your serial monitor to the same rate to view the output.

## Behavior

1. **RESET**: Fires a solenoid and homes each stepper in 1&nbsp;cm increments until its optical sensor is triggered.
2. **SEQUENCE**: Moves steppers, drives DC motors for fixed times, and fires both solenoids.
3. **STOP**: Interrupt halts any activity and returns to `IDLE`.

No microphone or SD card functionality is present in this project.
