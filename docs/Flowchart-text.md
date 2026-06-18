WavyPoint Robo — Flowchart textual walkthrough

This file explains the navigation flow in simple steps for screen readers and accessibility. Use the SVG in the README for a visual reference.

1. User / Startup
- Power on the robot or open the controller app.
- The robot performs a 10-second compass calibration (hard-iron calibration). It reports status via Bluetooth.

2. Sensor acquisition & smoothing
- Compass and GPS are read at regular intervals.
- GPS course and position are smoothed using an exponential moving average to reduce spikes.

3. Offset learning
- When the robot is moving faster than 3 km/h, GPS course is treated as the ground truth heading.
- The dynamic boresight offset is adjusted: either snapped on first movement or slowly converged (5% learning rate) otherwise.

4. Heading decision
- If absolute heading error &gt; 25°, the robot performs in-place rotation to reduce error quickly.
- Otherwise it uses proportional steering (Kp) for smooth course correction while cruising.

5. Waypoint tracking
- Distance to the active waypoint is computed via the Haversine formula.
- When within a small radius (default 5 m), the robot advances to the next waypoint.

6. Device & bridge
- A serial bridge (USB or Bluetooth) relays telemetry and commands between the robot and the host.
- Telemetry format (example): Hdg:045 GPS:042 Off:145 Err:-3 Dist:12.5

Notes for maintainers
- Keep the learning rate conservative to avoid overfitting to transient magnetic anomalies.
- Prefer visible telemetry output for faster debugging during new feature development.
