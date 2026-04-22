# NUControl Demos Setup
Below are instructions for setting up the hardware and software to run the NUControl demos.

## BOM Requirements

- **Motor Driver Board**: [HANDyDriver v2.0](https://github.com/cwoodhayes/HANDyDriver)
- **Microcontroller**: Teensy 4.0
- **Motor**: Maxon EC Motor 591476 (EC45 Flat)
- **Absolute Encoder**: AS5047P-TS_EK_AB v1.0 evaluation board
- **Current Sense**: Two inline current sensors (shunt + instrumentation amplifier configured for ~5 A/V gain)
- **Gate Driver**: HANDyDriver with MPQ6541GQKTE-AEC1-P (3-phase BLDC driver, 40V/8A)
- **Power Supply**: 18V at 2.5A nominal, 24V at 2.5A maximum
- **Communication**: USB cable for Teensy programming and Serial monitoring

## Hardware Setup

### Microcontroller & Driver Connections

**Teensy 4.0 Pin Configuration:**
- **Phase PWM pins**: Digital pins 3, 4, 5 (phases A, B, C respectively)
- **Driver enable pin**: Digital pin 2
- **SPI encoder chip select**: Digital pin 10
- **Current sense pins**: A0, A1 (ADC inputs for two of three phases)

### Encoder Board Wiring

*TODO - make a wiring diagram for this.*

Connect the AS5047P encoder board to the Teensy:
- **Power**: 3.3V from Teensy to encoder board 3V input (Note: the 3V Teensy output is connected to the encoder board's 5V input—this is what works in testing)
- **Ground**: GND to encoder board GND
- **SPI Interface**: 
  - MOSI → encoder board SPI input
  - MISO → encoder board SPI output
  - CLK → encoder board CLK
  - CS → Teensy pin 10

### Current Sensing

- **Phase A & B current sensors** are read via A0 and A1 on the Teensy (10-bit ADC)
- Each sensor should output 0–3.3V with a gain of ~5 A/V
- Sensors are configured with shunt resistors in the motor phase lines and instrumentation amplifiers (INA241A2IDDFR)
- Phase C current is inferred from Kirchhoff's current law (A + B + C = 0)

### Motor & Power

- **Motor phases**: Connected to gate driver outputs (phases A, B, C)
- **Power supply**: 18V, 2.5A nominal, 24V, 2.5A maximum connected to J1 (the large right-angle power connector on the driver board)

## Software Setup

### Prerequisites

1. **Install PlatformIO**: If not already installed, download the PlatformIO extension for VS Code
2. **udev rules for Teensy uploading** (Linux/Mac only):
   - Download the rules file: https://www.pjrc.com/teensy/00-teensy.rules
   - Place in `/etc/udev/rules.d/`
   - Without this, you'll get: "Teensy did not respond to a USB-based request to enter program mode"

### Firmware Build

The project uses **PlatformIO** to manage dependencies and build:

- **Board**: Teensy 4.0
- **Framework**: Arduino
- **Build optimization**: `-O2` (faster code than default `-Os`)
- **Monitor baud rate**: 115200

**Key dependencies:**
- TeensyTimerTool (for reliable hardware timers)
- Arduino SPI library

### IDE Keyboard Shortcuts (PlatformIO)

- `Ctrl+Alt+B`: Build
- `Ctrl+Alt+U`: Upload to Teensy

## Running the Demos

There are two demo environments configured in `platformio.ini`:

### 1. Position Sweep Demo (`position_sweep.cpp`)

https://github.com/user-attachments/assets/27d9cb95-7390-43a7-b38a-2bc345d98a3b

**What it does:** Sweeps the motor through 10 evenly-spaced angular targets (0–5.4 rad) in a back-and-forth pattern using PD torque control. Target advances approximately every 1 second.

**To run:**
1. Set the active environment to `position_sweep` in PlatformIO
2. Build and upload: `Ctrl+Alt+B`, then `Ctrl+Alt+U`
3. Open the Serial monitor at 115200 baud (`Ctrl+Shift+S`)
4. Motor will calibrate, then you should see messages like: `"Angular position target: X.X rad"`, and motor will sweep back and forth across the 10 target positions.

**Expected behavior:** Motor smoothly moves between positions without overshooting significantly.

### 2. One-Shot Demo (`one_shot.cpp`)

https://github.com/user-attachments/assets/84293020-8c51-4c20-a827-f513a655a712

**What it does:** Applies constant torque to the motor until it exceeds a velocity threshold (240 RPS), then stops. Includes an overspeed governor for safety.

**To run:**
1. Set the active environment to `one_shot` in PlatformIO
2. Build and upload: `Ctrl+Alt+B`, then `Ctrl+Alt+U`
3. Open the Serial monitor at 115200 baud (`Ctrl+Shift+S`)
4. Motor will calibrate, then accelerate briefly under constant torque (for ~1s) and spin down at overspeed.

**Known issues & debugging:**
- **Current sensor saturation**: The sensor may read near its maximum under high acceleration. Monitor the serial output for "Current Sensor Saturated!" warnings
- **Instability at high speeds**: Setting the PSU to (24V, 2.5A) and `MAX_VEL` to 300 RPS will trigger the instability; motor may fail to shut down cleanly and draw excessive current. This can be reproduced using the one_shot demo
- **Velocity sensing**: Ensure the encoder is reading correctly (check `get_shaft_velocity()` output). If velocity reads zero, verify encoder wiring and SPI communication

### Power Supply Checklist

Before running either demo:
- [ ] 18V power supply is connected to J1
- [ ] Supply is set to 2.5A current limit
- [ ] Motor is mechanically free to spin (no jamming)
- [ ] All wiring is correct

### Serial Output Interpretation

**Position Sweep output:**
```
Angular position target: 0.0 rad
Angular position target: 0.6 rad
...
```

**One-Shot output:**
```
Shaft Angle: X.XX    Shaft Velocity: Y.YY    T Setpoint: Z.ZZ
[Repeats at 1 Hz]
Overspeed! Stopping motor. (vel=240.0)
```

### Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Upload fails with "Teensy did not respond" | Missing udev rules | Install `/etc/udev/rules.d/00-teensy.rules` |
| Encoder reads zero/garbage | SPI wiring or CS pin issue | Verify SPI connections; check chip select pin 10 |
| Motor doesn't move | Enable pin (2) not toggling | Check gate driver initialization; verify firmware compiled without errors |
| Current spikes/saturation warnings | Sensor miscalibration or excessive load | Reduce target torque; verify 5 A/V gain is correct |