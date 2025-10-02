# Newton CFV Controller

Codebase for the Newton countertop flavoring (CFV) motor controller running on
a Raspberry Pi. The repository now focuses on the streamlined `simple_control`
interface for driving the auger BLDC motor and monitoring its speed feedback in
real time.

## Hardware
- Raspberry Pi 4
- ELECROW 5" HDMI display
- MC-BLDC-300-S motor driver with BLDC-J57-107-36-4800-04 motor
- Lid interlock switch wired to GPIO 25

Driver wiring highlights:
- PWM command: GPIO 18
- Enable (active low): GPIO 20
- Brake (active low): GPIO 19
- Tach feedback: GPIO 16
- Direction: GPIO 12 (held HIGH for forward)

## Running the Touchscreen UI
```bash
./run_newton_ui.sh
```
The launcher applies the same Qt environment variables used on the Pi and will
prefer the `/home/admin/ice_shaver_env` virtualenv if present.

## Repository Layout
- `motor.py` – reusable BLDC helper with watchdog and smoothed RPM reporting.
- `simple_control/` – PyQt UI and motor wrapper used by the touchscreen.
- `run_newton_ui.sh` – convenience launcher for deployments.

## Development Notes
- Ensure the `pigpiod` daemon is active before running the UI.
- `motor.py` and `simple_control/motor_rpm.py` rely on pigpio for pulse
  callbacks; the smoothing constants can be tuned inside those modules if the
  hardware setup changes.
- Add a virtualenv-specific `.env` or adjust `run_newton_ui.sh` if your Python
  interpreter lives elsewhere.

