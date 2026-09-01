# Contributing

Thanks for helping improve WavyPoint Robo.

## Before opening a pull request

1. Fork and clone the repository.
2. Open `wavy.c` in the Arduino IDE, or copy it to an `.ino` sketch if required by your IDE.
3. Install the PRIZM, Adafruit QMC5883P, and TinyGPSPlus libraries.
4. Create a focused branch such as `fix/gps-timeout` or `feature/resume-command`.

## Code changes

- Keep hardware constants near the top of the firmware and explain non-obvious values.
- Prefer small functions and descriptive names.
- Do not commit real private routes or personal location data as sample waypoints.
- Update the README and flowchart when control flow, telemetry, or setup changes.

## Testing

In the pull request, state:

- the board, sensors, GPS, Bluetooth module, motors, and library versions used;
- whether compilation succeeded and for which board target;
- the bench and outdoor tests performed;
- expected and observed behavior;
- any part that could not be tested on physical hardware.

Raise the drive wheels for initial motor testing and include evidence that the Bluetooth stop command works when changing movement logic.

## Issues

Search existing issues first. Bug reports should include the firmware commit, hardware setup, wiring or pin changes, reproduction steps, telemetry output, and expected behavior. Remove precise GPS coordinates if they reveal a private location.
