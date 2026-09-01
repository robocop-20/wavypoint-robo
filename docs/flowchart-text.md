# Navigation flowchart: text version

This is the accessible text equivalent of [`assets/flowchart.svg`](../assets/flowchart.svg).

1. On startup, the firmware initializes Serial, Bluetooth, GPS, PRIZM, and the QMC5883P compass.
2. If compass initialization fails, the motors remain unused and `Compass Fail` is reported repeatedly.
3. Otherwise, the robot waits eight seconds while the operator rotates it through 360 degrees. The observed minimum and maximum X/Y readings are used to calculate hard-iron offsets.
4. The main loop checks Bluetooth. Receiving `S` or `s` disables navigation and stops both motors.
5. GPS bytes are decoded continuously. Control calculations run whenever TinyGPSPlus reports an updated location.
6. The GPS latitude and longitude are smoothed with an exponential moving average.
7. The firmware calculates Haversine distance and initial bearing from the smoothed position to the active waypoint.
8. Compass heading is refreshed every 100 ms. Encoder heading changes may be added when `PURE_COMPASS_MODE` is `false`.
9. If the waypoint is less than five metres away, the robot stops, waits two seconds, and advances to the next waypoint. After the final waypoint, navigation is disabled.
10. If the waypoint is not reached:
    - an absolute heading error over 25 degrees causes a turn in place;
    - otherwise, proportional correction is applied while both motors drive forward.
11. Heading, target bearing, heading error, and distance are sent over Bluetooth on every processed GPS update.

The current firmware does not learn compass offset from GPS course and has no companion-app connect/record/start sequence.
