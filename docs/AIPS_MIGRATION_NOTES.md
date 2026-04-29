# AIPS notes for term3_robot

Useful pieces imported/adapted from `AIPS`:

- WiFi/UDP bring-up pattern: scan, connect, open UDP port, reply to sender.
- PID controller: compact target/gain/integral-limit loop for motor velocity.
- Cooperative task scheduler: small periodic task runner for motor, UDP, status loops.
- Encoder velocity calculation: interrupt count delta -> rad/s with low-pass filtering.
- Manual DFU upload workaround for Arduino GIGA R1 on this Windows machine.

Pieces intentionally not copied directly:

- `Modulino` peripherals: UNO R4-specific and not needed until hardware is chosen.
- `Motoron` driver binding: useful if the robot uses Pololu Motoron, but the starter uses generic PWM+DIR pins because `term3_robot` does not yet define a motor driver.
- Master/slave I2C split: AIPS is a two-board robot; this starter begins as one GIGA project.
- OLED, LiDAR, QTR sensor code: keep these as references in AIPS until the new robot hardware layout is fixed.

Next hardware decisions:

1. Pick the motor driver board and update `EncoderMotor::write_power()` if it is not PWM+DIR.
2. Update `include/robot_config.hpp` with real pins and WiFi credentials.
3. If you use the Pololu Motoron, copy the AIPS `motor.hpp/.cpp` style instead of this generic motor class.
4. Add packet definitions for your controller app once command format is known.
