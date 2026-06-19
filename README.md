<div align="center">

# WavyPoint Robo

**Autonomous waypoint navigation on PRIZM + Arduino.**
GPS + compass fusion with automatic boresight offset learning, Haversine-based distance tracking, and live Bluetooth telemetry.

[![Platform](https://img.shields.io/badge/platform-PRIZM%20%2F%20Arduino-1D9E75.svg)](#hardware)
[![Status](https://img.shields.io/badge/status-active-9FE1CB.svg)](#roadmap)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-EF9F27.svg)](CONTRIBUTING.md)

</div>

---

A robot that drives itself between GPS coordinates and corrects its own compass error while it moves — no manual calibration step, no external IMU. It learns its sensor's mounting offset from the GPS ground-truth heading in real time, then runs a hybrid turn-in-place / proportional steering loop to track each waypoint and advance automatically.

```
Hdg:045 GPS:042 Off:145 Err:-3 Dist:12.5m  Wp:1/3
Hdg:047 GPS:043 Off:144 Err:-1 Dist:10.1m  Wp:1/3
Hdg:049 GPS:049 Off:143 Err:+1 Dist:7.2m   Wp:1/3
Hdg:050 GPS:050 Off:143 Err:0  Dist:5.8m   Wp:1/3
→ Waypoint 1 reached. Advancing...
```
*Live telemetry streamed over Bluetooth — every field explained in [Telemetry](#telemetry).*

## Why this exists

Most hobby GPS-nav builds either skip compass calibration entirely (heading drifts within minutes) or require a manual figure-eight calibration ritual before every run. WavyPoint Robo does neither — it cross-references the compass against GPS course while the robot is in motion above 3 km/h, and continuously corrects its boresight offset on the fly. The result is a heading estimate that gets *more* accurate the longer it runs, with zero setup steps.

It's also small enough to read end-to-end in one sitting and built on parts you can source from a single electronics order — useful as a reference implementation for anyone teaching or learning sensor fusion, control loops, or embedded navigation.

## Features

- **Waypoint autonomy** — Haversine great-circle distance, automatic advance on arrival
- **Self-correcting heading** — boresight offset learned continuously from GPS course, no calibration step
- **GPS smoothing** — exponential moving average filters noisy fixes before they reach the control loop
- **Hybrid steering** — turn-in-place when heading error is large, proportional correction when on-course
- **Live telemetry** — full state streamed over Bluetooth at run time for debugging and demos

## Architecture

```
Sensor read ──▶ EMA smoothing ──▶ Offset learning ──▶ Heading control ──▶ Waypoint advance
(QMC5883P,        (GPS + heading      (GPS course           (turn-in-place /     (Haversine
 UBLOX GPS)         filter)             @ > 3 km/h)            proportional)        distance)
```

See [`assets/flowchart.svg`](assets/flowchart.svg) for the full diagram, or [`docs/flowchart-text.md`](docs/flowchart-text.md) for a text-only walkthrough.

## Hardware

| Component | Model | Role |
|---|---|---|
| Controller | PRIZM | Motor control + main compute |
| Compass | QMC5883P | Magnetic heading, with offset learning |
| GPS | UBLOX | Position, course, speed |
| Drive | DC motors | Differential drive |
| Telemetry | HC-05 | Bluetooth serial, 9600 baud |
| Encoders | *(optional)* | Odometry cross-check |

## Quickstart

```bash
git clone https://github.com/robocop-20/wavypoint-robo
cd wavypoint-robo
```

1. Open `firmware/` in the Arduino IDE
2. Install libraries via **Sketch → Include Library → Manage Libraries**: PRIZM support, `Adafruit QMC5883P`, `TinyGPSPlus`
3. Edit the waypoint list in firmware (see [Configuration](#configuration))
4. Upload to the PRIZM controller and power the robot
5. Connect over Bluetooth and watch for `Ready. Auto-Align Active.` — navigation starts automatically

## Telemetry

| Field | Meaning |
|---|---|
| `Hdg` | Compass heading (0–359°) |
| `GPS` | GPS course |
| `Off` | Learned boresight offset |
| `Err` | Heading error to target |
| `Dist` | Distance to current waypoint (m) |
| `Wp` | Current waypoint index / total |

## Configuration

**GPS baud rate** — match to your module:
```cpp
GPS_Serial.begin(9600);
```

**Compass behavior:**
```cpp
const bool AUTO_LEARN_OFFSET = true;
const float declinationDeg = -0.6;   // set for your location
```

**Navigation tuning:**
```cpp
const int CRUISE_SPEED = 50;
const int TURN_SPEED   = 45;
const float WHEEL_RADIUS = 0.045;    // meters
const float WHEEL_BASE   = 0.34;     // meters
```

**Waypoints** (lat, lon):
```cpp
NavPoint waypoints[] = {
  {17.780223, 83.375458},
  {17.780277, 83.375477},
  {17.780322, 83.375368}
};
```

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Compass heading is wrong/inverted | Check `SENSOR_AXIS_FORWARD` / `SENSOR_AXIS_RIGHT` mapping in firmware |
| GPS never gets a fix / no updates | Verify baud rate and wiring; confirm outdoor sky view |
| Offset never converges | Robot must exceed 3 km/h during the learning phase |
| Robot spins instead of driving straight | Check motor direction/invert settings |

## Roadmap

- [ ] IMU integration for GPS-denied / indoor operation
- [ ] Web telemetry dashboard (live map + heading plot)
- [ ] Multi-waypoint upload over Bluetooth (no reflash required)
- [ ] Obstacle-aware path planning

Want to take one of these on? See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Contributing

Issues and PRs are welcome — see [`CONTRIBUTING.md`](CONTRIBUTING.md) for setup and PR guidelines. A short demo clip or screenshot with hardware-related PRs helps a lot.
