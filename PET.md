# Pet fork: what is different

[По-русски](PET.ru.md)

A fork of [kaiaai/firmware](https://github.com/kaiaai/firmware) for a small home robot: ESP32 DevKit (30 pins), L298N driver, two JGA25-370B gear motors with encoders, LD19 lidar, 4S battery. The "mind" of the robot runs on a home PC; that part lives in a separate project.

**Status: builds, logic is tested on a PC, not yet tested on hardware.** The motors are still in the mail. Treat the thresholds as starting values.

## Why

The original firmware stops the motors only when Wi-Fi is lost. Three cases were not covered:

| Case | What happened before | Now |
|---|---|---|
| The PC hangs, Wi-Fi stays up | The robot keeps driving at the last commanded speed | No `/cmd_vel` for 0.5 s — motors stop |
| The robot runs into something | The PID raises power to the limit, motor and driver heat up | Motor is driven but the wheel does not turn for 1 s — stop, commands are ignored for 2 s. A reversed encoder is caught the same way |
| The battery runs down | Voltage is only reported in telemetry | Three steps: report, then motors and lidar off, then deep sleep with a wake-up every minute to re-check |

Stall and battery events are published to `/rosout` as `SAFETY: ...` warnings, so the PC side can react.

There is also one fix to existing code: in `IN1_IN2` mode the hard brake wrote to a pin that was still attached to a PWM channel, so the motor could stay on its last duty cycle. See `brakeMotor()` in `motors.h`.

## Settings

Everything is in `kaiaai-esp32/data/config.yaml`, section `safety`. A value of `0` switches a check off. Battery protection is off unless thresholds are set, so the original configs behave as before.

```yaml
safety:
  cmd_vel:
    timeout: 0.5
  stall:
    timeout: 1.0
    rpm_min: 3.0
    pwm_min: 0.15
    hold: 2.0
  battery:
    low: 14.0
    stop: 13.2
    sleep: 12.8
    hysteresis: 0.5
    delay: 3.0
    wake: 60
```

The battery steps back up only at the threshold plus `hysteresis`, and a new level has to last `delay` seconds — otherwise the voltage sag under motor load would switch it back and forth. If the measured voltage is below half of the `sleep` threshold, the firmware assumes there is no battery (USB power on the bench) and leaves battery protection inactive.

Note on the config parser: comments are allowed only at the start of a line, not after a value.

## Build

The firmware needs Arduino core 2.0.x (ESP-IDF 4.4); core 3.x is not supported. A PlatformIO project is included:

```
pio run              # build
pio run -t upload    # flash the firmware
pio run -t uploadfs  # flash kaiaai-esp32/data (config.yaml, web page)
```

On Windows keep the project in a path without non-Latin characters, otherwise the ESP32 toolchain fails.

## Test on a PC

`test_host/run_test.ps1` compiles `safety.h` against small stand-ins for the Arduino and ROS parts and runs it through the scenarios: lost commands, stall, one blocked wheel, reversed encoder, battery steps with hysteresis, boot on an empty battery, and the 32-bit microsecond counter wrap. It needs `zig` as a C++ compiler (`pip install ziglang`).

## Still to do on hardware

- Calibrate the battery divider against a multimeter (`adc.battery.attenuation`).
- Measure encoder pulses per revolution and wheel track.
- Pick `stall.pwm_min` for these motors.
- Confirm on raised wheels that all three protections trigger.
