# scripts

## motor_monitor.py

Bridges the Teensy's data serial port (SerialUSB1) to PlotJuggler via UDP for live motor telemetry plots.

**Dependencies:** `pip install pyserial`, [PlotJuggler](https://appimage.github.io/PlotJuggler/)

```bash
python3 scripts/motor_monitor.py /dev/ttyACM1
```

`Serial` (console) and `SerialUSB1` (data) enumerate as separate ports — typically `ttyACM0` and `ttyACM1`. Run `ls /dev/ttyACM*` before and after plugging in to confirm which is which.

**First-time PlotJuggler setup:** Streaming → UDP Server → port 9870 → Start, arrange your plots, then File → Save Layout → `scripts/plotjuggler_layout.xml`. Commit that file and it loads automatically on every subsequent run.

**Options:**

| Flag | Description |
|---|---|
| `--udp-port PORT` | UDP port PlotJuggler listens on (default: 9870) |
| `--layout FILE` | PlotJuggler layout file (default: `scripts/plotjuggler_layout.xml`) |
| `--no-plotjuggler` | Bridge only, don't launch PlotJuggler |
