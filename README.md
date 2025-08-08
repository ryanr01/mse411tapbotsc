# Arduino Robot Control

This repository contains a single `main.c` sketch that drives a small robot using an Arduino-compatible board. The sketch demonstrates:

- **Two stepper motors** driven by DRV8825 drivers (step and direction lines only)
- **Two DC motors** with encoders through L298N drivers
- **Two solenoids** via L298N
- **Three push buttons** (START, RESET, STOP) configured as interrupts
- A `switch`/`enum` state machine (`IDLE`, `RESET`, `SEQUENCE`, `DONE`)

## Building

Compile and upload the sketch with your preferred Arduino toolchain. For example, using `arduino-cli`:

```bash
arduino-cli compile --fqbn <board> .
arduino-cli upload --port <port> --fqbn <board> .
```

The code relies only on the Arduino core and runs the sequence inside the `loop()` function.

## Behavior

1. **RESET**: Fires a solenoid and homes each stepper in 1&nbsp;cm increments until its optical sensor is triggered.
2. **SEQUENCE**: Moves steppers, drives DC motors for set distances, and fires both solenoids.
3. **STOP**: Interrupt halts any activity and returns to `IDLE`.

No microphone or SD card functionality is present in this project.
