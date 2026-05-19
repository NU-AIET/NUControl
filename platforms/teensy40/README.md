# NUControl — Teensy 4.0 Platform

This directory is a self-contained PlatformIO project targeting the Teensy 4.0. It contains all hardware-specific code (PWM driver, SPI encoder, current sensing, timer management) and the demo applications.

## Dependencies

- [HANDyDriver](https://github.com/cwoodhayes/HANDyDriver) motor driver board
- Maxon EC45 Flat brushless motor + AS5047P absolute encoder
- PlatformIO with Teensy platform installed

## Building

Open this directory (`platforms/teensy40/`) as the PlatformIO project root — not the repo root.

```bash
# From platforms/teensy40/
pio run --environment position_sweep   # build position sweep demo
pio run --environment one_shot         # build one-shot torque demo
pio run --environment position_sweep --target upload   # build and flash
```

Or use the PlatformIO VS Code extension (`Ctrl+Alt+B` to build, `Ctrl+Alt+U` to upload).

The portable math library in `../../core/` is pulled in automatically via the `-I../../core` build flag.

## Demos

See [demos/DEMOS.md](demos/DEMOS.md) for full hardware setup, wiring, and run instructions.
