Thank you for contributing to WavyPoint Robo — we want this project to be friendly to beginners and useful to embedded developers. Please read these guidelines before opening a pull request.

Getting your dev environment

1. Fork the repo and clone your fork.
2. Open the firmware/ folder in Arduino IDE.
3. Install any missing libraries via Arduino Library Manager (Adafruit QMC5883P, TinyGPSPlus, PRIZM libs).

Branching & PRs

- Create a descriptive branch: feature/your-feature or fix/short-description
- Keep PRs small and focused
- Include a short description, testing steps, and a screenshot/GIF of the change if relevant

Coding style

- Keep changes clear and documented. Add comments for hardware-specific magic numbers.
- For C code, prefer clear naming and small functions. Add TODOs with context.

Testing

- Manually test firmware on a PRIZM device when possible (include steps in the PR description).
- If adding pure docs or examples, ensure links and code blocks render correctly on GitHub.

Issues

- Before creating an issue, search existing issues and README/Troubleshooting.
- For bug reports: include hardware used, firmware version (commit), steps to reproduce, and expected vs actual behavior.

License & attribution

- Keep license as-is. By contributing, you agree your contributions follow the repository license.

Thank you — small PRs with clear demos get reviewed fastest.