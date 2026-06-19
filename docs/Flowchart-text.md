# Navigation flowchart — text walkthrough

A plain-text version of [`assets/flowchart.svg`](../assets/flowchart.svg), for accessibility or if the image doesn't render.

## 1. User

- Power on the robot, or open the companion app
- Sequence: **Connect → Record → Start**

## 2. Control loop (runs continuously on the PRIZM)

1. **Sensor read → EMA smoothing**
   Raw compass and GPS readings are filtered with an exponential moving average to remove noise before they reach the control logic.

2. **Offset learning**
   While speed exceeds 3 km/h, the GPS course (ground truth) is compared against the raw compass heading. The difference is learned as a boresight offset and applied going forward — this is what lets the robot self-correct without a manual calibration step.

3. **Heading decision**
   - If heading error is large: turn in place to face the target
   - If heading error is small: apply proportional steering correction while driving forward

4. **Waypoint advance & stop logic**
   Haversine distance to the current waypoint is checked each cycle. Below the arrival threshold, the robot advances to the next waypoint in the list, or stops if it was the last one.

## 3. Device / bridge

- **Serial bridge (USB / Bluetooth)** — connects to PRIZM firmware, which drives the motors directly
- **GPS module** — supplies course and speed, used both for navigation and for offset learning
- **Compass** — supplies raw heading, corrected by the learned offset before use

## Telemetry line format

```
Hdg:045 GPS:042 Off:145 Err:-3 Dist:12.5
```

| Field | Meaning |
|---|---|
| `Hdg` | Compass heading (0–359°) |
| `GPS` | GPS course |
| `Off` | Learned boresight offset |
| `Err` | Heading error to target |
| `Dist` | Distance to waypoint (meters) |
