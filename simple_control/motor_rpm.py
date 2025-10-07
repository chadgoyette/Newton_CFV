"""Motor controller helper for the simplified ice shaver UI.

This module keeps all of the hardware specific behaviour for the auger motor in
one place.  The touchscreen UI simply asks for a percentage (0-100 %) and the
motor helper handles the details of:

* configuring the pigpio daemon and the GPIO pins
* translating the percentage into an actual PWM signal
* sampling the driver feedback pulses to compute auger RPM
* enforcing the lid interlock and brake logic
* performing an automatic short reverse at the end of every run

Every section below is heavily commented so it is clear what each line achieves
and how the state machine fits together.
"""

from __future__ import annotations

# --- Standard library imports -------------------------------------------------
import threading  # Background sampler runs in its own thread
import time  # Timing utilities for sleeps and timestamps
from dataclasses import dataclass  # Lightweight container for GPIO pins
from typing import Optional  # Type hints for optional attributes

# --- Third-party hardware interfaces -----------------------------------------
import pigpio  # Primary GPIO/PWM access (via the pigpiod daemon)
import RPi.GPIO as GPIO  # Used for the simple interlock input

# --- Calibration and timing constants ----------------------------------------
MOTOR_RPM_PER_HZ = 5.0        # 1 Hz of feedback ≈ 5 motor RPM (bench measurement)
ROTOR_CORRECTION = 1.0364     # Scales motor RPM to auger RPM after gearing
COOLDOWN_WINDOW_SEC = 1.0     # Additional sampling time after a run completes
BRAKE_SETTLE_SEC = 0.3        # Delay between braking and the reverse pulse
SAMPLE_TIME_DEFAULT = 0.05    # Shorter accumulation window for near realtime updates
EMA_ALPHA_PULSE = 0.45        # Weight for instantaneous (per pulse) measurements
EMA_ALPHA_WINDOW = 0.25       # Weight for windowed pulse counts
EMA_ALPHA_TIMEOUT = 0.15      # Weight when pulses disappear (decay toward zero)
WATCHDOG_TIMEOUT_MS = 150     # Flag missing pulses quickly without excess chatter


# --- PWM / GPIO pin map -------------------------------------------------------
@dataclass
class MotorPins:
    """Groups all GPIO assignments used by the auger motor."""

    pwm: int              # PWM output that carries the 0-100 % command to the driver
    speed_feedback: int   # Tach pulse input from the driver (used to measure RPM)
    enable: int           # Driver enable pin (active LOW)
    brake: int            # Brake input (LOW = engaged, HIGH = released)
    direction: int = 12   # Direction control pin (0 = forward, 1 = reverse)
    interlock: int = 25   # Lid interlock input (HIGH = lid open)


class RPMControlledMotor:
    """Encapsulates the BLDC driver logic for the touchscreen UI."""

    def __init__(
        self,
        pins: MotorPins,
        motor_pulley_diameter: float = 22.0,
        rotor_pulley_diameter: float = 95.0,
        sample_time: float = SAMPLE_TIME_DEFAULT,
        bypass_interlock: bool = False,
    ) -> None:
        """Store configuration and prepare GPIO state."""

        # Remember the wiring and calibration values supplied by the caller.
        self.pins = pins
        self.motor_pulley_diameter = motor_pulley_diameter
        self.rotor_pulley_diameter = rotor_pulley_diameter
        self.sample_time = max(0.02, sample_time)  # 20ms minimum for stable readings
        self._bypass_interlock = bypass_interlock

        # Initialise runtime state used by the control loop.
        self._pulse_count = 0  # Number of tach pulses captured since last snapshot
        self._pulse_lock = threading.Lock()  # Protects pulse counter access
        self._stop_event = threading.Event()  # Signals the sampling thread to exit
        self._control_thread: Optional[threading.Thread] = None  # Background sampler
        self._target_percent = 0.0  # Forward command requested by the UI
        self._current_percent = 0.0  # Command currently applied to pigpio
        self._raw_rotor_rpm = 0.0  # Latest unsmoothed auger RPM
        self._filtered_rotor_rpm = 0.0  # Exponentially smoothed auger RPM for display
        self._last_rotor_rpm = 0.0  # Backwards compatible name for smoothed RPM
        self._ran_forward = False  # True once we have issued a non-zero forward PWM
        self._last_pulse_tick: Optional[int] = None  # pigpio tick of the last tach pulse
        self._cycle_end_time: Optional[float] = None  # When the cycle should stop
        self._cycle_start_time: Optional[float] = None  # When the cycle began
        self.was_interrupted = False  # True if the lid interlock tripped mid run
        self._cooldown_deadline: Optional[float] = None  # When to stop cooldown sampling
        self._brake_engaged = True  # Cached brake state so the UI stays in sync
        self._sampling_enabled = False  # Gate tach updates when motor should stay idle
        self._watchdog_armed = False  # Tracks whether pigpio watchdog is active

        # Configure the interlock input using the simple RPi.GPIO library.
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pins.interlock, GPIO.IN)

        # Connect to pigpiod and configure the hardware pins we drive.
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon.")
        self._callback = None
        self._initialize_pigpio()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def start_cycle(self, target_percent: float, duration_seconds: float) -> None:
        """Begin a forward run at the requested percentage for the given duration."""

        # Prevent overlapping runs – the UI should only start a new cycle when idle.
        if self._control_thread and self._control_thread.is_alive():
            raise RuntimeError("Motor cycle already running.")

        # Validate input arguments early to surface issues back to the UI.
        if not (0 <= target_percent <= 100):
            raise ValueError("Target percent must be between 0 and 100.")
        if duration_seconds < 0:
            raise ValueError("Duration must be >= 0 seconds.")

        # Reset state for a fresh run.
        self._stop_event.clear()
        self.was_interrupted = False
        self._target_percent = float(target_percent)
        self._ran_forward = False
        self._last_pulse_tick = None
        self._cycle_start_time = time.time()
        self._cycle_end_time = (
            self._cycle_start_time + duration_seconds if duration_seconds > 0 else None
        )
        self._cooldown_deadline = None
        self._sampling_enabled = True
        self._filtered_rotor_rpm = 0.0
        self._raw_rotor_rpm = 0.0
        self._last_rotor_rpm = 0.0

        # Arm the hardware – brake released, enable asserted, callback primed.
        if not self._arm_motor():
            raise RuntimeError("Failed to arm motor (check interlock and wiring).")

        # Ensure we start counting pulses from zero for this cycle.
        with self._pulse_lock:
            self._pulse_count = 0

        # Apply the requested command and launch the sampling thread.
        self._apply_pwm(self._target_percent)
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    def stop_cycle(self) -> None:
        """Request the current cycle to stop and clean up if the thread is idle."""

        self._stop_event.set()
        thread = self._control_thread
        if thread and thread.is_alive():
            thread.join()
            return
        self._disarm_motor()

    def shutdown(self) -> None:
        """Fully release hardware resources (used on program exit)."""

        try:
            self.stop_cycle()
        finally:
            if self._callback is not None:
                self._callback.cancel()
        self._set_watchdog(False)
        self.pi.stop()

    # ------------------------------------------------------------------
    # State helpers used by the UI layer
    # ------------------------------------------------------------------
    @property
    def last_rotor_rpm(self) -> float:
        """Expose the most recent auger RPM measurement."""

        return self._last_rotor_rpm

    @property
    def remaining_cycle_time(self) -> Optional[float]:
        """How much time is left in the current run (seconds)."""

        if self._cycle_end_time is None:
            return None
        remaining = self._cycle_end_time - time.time()
        return max(0.0, remaining)

    @property
    def cycle_elapsed_time(self) -> Optional[float]:
        """How long the run has been active (seconds)."""

        if self._cycle_start_time is None:
            return None
        return max(0.0, time.time() - self._cycle_start_time)

    @property
    def is_running(self) -> bool:
        """Whether the control thread is currently active."""

        return self._control_thread is not None and self._control_thread.is_alive()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _pulse_callback(self, gpio: int, level: int, tick: int) -> None:
        """pigpio callback – counts tach pulses for later RPM calculation."""

        # pigpio already filters for rising edges, but guard against watchdog callbacks.
        if level == pigpio.TIMEOUT:
            if not self._sampling_enabled:
                return
            self._last_pulse_tick = None
            self._update_rotor_rpm(0.0, EMA_ALPHA_TIMEOUT)
            return
        if level not in (pigpio.RISING_EDGE, 1):
            return
        if not self._sampling_enabled:
            return

        # Accumulate a windowed pulse count for the sampling loop.
        with self._pulse_lock:
            self._pulse_count += 1

        # Use the tick delta to refresh the instantaneous RPM used by the UI.
        if self._last_pulse_tick is not None:
            delta_us = pigpio.tickDiff(self._last_pulse_tick, tick)
            if delta_us > 0:
                frequency = 1_000_000.0 / delta_us
                rotor_rpm = self._rotor_rpm_from_frequency(frequency)
                self._update_rotor_rpm(rotor_rpm, EMA_ALPHA_PULSE)

        self._last_pulse_tick = tick

    def _arm_motor(self) -> bool:
        """Release the brake and enable the driver, respecting the interlock."""

        if self._interlock_open():
            self.was_interrupted = True
            return False

        if self.pi is None or not self.pi.connected:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                return False
            self._initialize_pigpio()

        self.pi.write(self.pins.enable, 0)  # Active low – enables the driver
        self._set_brake(False)  # Release the brake for forward motion
        time.sleep(0.25)  # Let hardware settle
        self._set_watchdog(True)
        return True

    def _disarm_motor(self) -> None:
        """Stop applying PWM, engage the brake, and schedule the reverse."""

        had_motion = self._ran_forward or self._last_rotor_rpm > 5
        self._apply_pwm(0)  # Stop the forward command
        self._set_brake(True)  # Engage brake immediately
        time.sleep(BRAKE_SETTLE_SEC)  # Hold the brake before reverse
        self.pi.write(self.pins.enable, 1)  # Disable driver (inactive high)
        self._target_percent = 0.0
        self._current_percent = 0.0
        self._filtered_rotor_rpm = 0.0
        self._raw_rotor_rpm = 0.0
        self._last_rotor_rpm = 0.0
        self._cooldown_deadline = None
        self._sampling_enabled = False
        self._set_watchdog(False)
        if had_motion and not self.was_interrupted:
            self._reverse_spin()

    def _interlock_open(self) -> bool:
        """Return True when the lid switch is open."""

        if self._bypass_interlock:
            return False
        return GPIO.input(self.pins.interlock) == GPIO.HIGH

    def io_snapshot(self) -> dict[str, float | int | bool]:
        """Collect the current GPIO and telemetry values for the UI."""

        snapshot: dict[str, float | int | bool] = {}
        if self.pi is None or not self.pi.connected:
            return snapshot

        try:
            pwm_raw = self.pi.get_PWM_dutycycle(self.pins.pwm)
        except pigpio.error:
            pwm_raw = 0

        snapshot["pwm_raw"] = pwm_raw
        snapshot["pwm_percent"] = (pwm_raw / 255.0) * 100.0
        snapshot["target_percent"] = self._target_percent
        snapshot["current_percent"] = self._current_percent
        snapshot["motor_rpm"] = (
            self._last_rotor_rpm
            * (self.rotor_pulley_diameter / self.motor_pulley_diameter)
            / ROTOR_CORRECTION
        )
        snapshot["motor_rpm_raw"] = (
            self._raw_rotor_rpm
            * (self.rotor_pulley_diameter / self.motor_pulley_diameter)
            / ROTOR_CORRECTION
        )

        def safe_read(pin: int) -> int:
            try:
                return self.pi.read(pin)
            except pigpio.error:
                return -1

        snapshot["enable"] = safe_read(self.pins.enable)
        snapshot["brake"] = safe_read(self.pins.brake)
        snapshot["direction"] = safe_read(self.pins.direction)
        snapshot["speed_feedback"] = safe_read(self.pins.speed_feedback)

        try:
            snapshot["interlock"] = GPIO.input(self.pins.interlock)
        except RuntimeError:
            snapshot["interlock"] = -1

        snapshot["bypass_interlock"] = self._bypass_interlock
        return snapshot

    def _apply_pwm(self, duty_cycle: float) -> None:
        """Send a new duty cycle to the driver (clamped to 0-100)."""

        duty_cycle = max(0.0, min(100.0, duty_cycle))
        pwm_value = int((duty_cycle / 100.0) * 255)
        self.pi.set_PWM_dutycycle(self.pins.pwm, pwm_value)
        self._current_percent = duty_cycle
        if duty_cycle > 0:
            self._ran_forward = True

    def _set_brake(self, engaged: bool) -> None:
        """Drive the brake line and remember its current state."""

        if self.pi is None or not self.pi.connected:
            return
        level = 0 if engaged else 1  # Brake is active low
        if self._brake_engaged == engaged and self.pi.read(self.pins.brake) == level:
            return
        self.pi.write(self.pins.brake, level)
        self._brake_engaged = engaged

    def _reverse_spin(self) -> None:
        """Briefly spin the auger backwards to relieve any binding."""

        if self.was_interrupted or self.pi is None or not self.pi.connected:
            return

        try:
            self.pi.write(self.pins.direction, 1)  # Switch to reverse
            self.pi.write(self.pins.enable, 0)  # Enable driver
            self._set_brake(False)  # Release brake for reverse pulse
            self.pi.set_PWM_dutycycle(self.pins.pwm, int(0.3 * 255))  # 30 % reverse
            time.sleep(0.5)
        finally:
            self.pi.set_PWM_dutycycle(self.pins.pwm, 0)  # Stop PWM
            self._set_brake(True)  # Engage brake again
            self.pi.write(self.pins.enable, 1)  # Disable driver
            self.pi.write(self.pins.direction, 0)  # Restore forward direction
            self._ran_forward = False
            self._sampling_enabled = False
            self._last_rotor_rpm = 0.0
            self._last_pulse_tick = None
            with self._pulse_lock:
                self._pulse_count = 0

    def _initialize_pigpio(self) -> None:
        """Configure all pigpio controlled pins and register the callback."""

        self.pi.set_mode(self.pins.pwm, pigpio.OUTPUT)
        self.pi.set_mode(self.pins.speed_feedback, pigpio.INPUT)
        self.pi.set_mode(self.pins.enable, pigpio.OUTPUT)
        self.pi.set_mode(self.pins.brake, pigpio.OUTPUT)
        self.pi.set_mode(self.pins.direction, pigpio.OUTPUT)
        self.pi.set_pull_up_down(self.pins.speed_feedback, pigpio.PUD_UP)
        self.pi.set_glitch_filter(self.pins.speed_feedback, 100)

        self.pi.write(self.pins.enable, 1)  # Keep motor disabled until armed
        self.pi.write(self.pins.brake, 0)   # Brake engaged by default
        self._brake_engaged = True
        self.pi.write(self.pins.direction, 0)  # Forward direction by default
        self.pi.set_PWM_frequency(self.pins.pwm, 4000)

        if self._callback is not None:
            self._callback.cancel()
        self._callback = self.pi.callback(
            self.pins.speed_feedback, pigpio.RISING_EDGE, self._pulse_callback
        )
        self._set_watchdog(False)

    def _snapshot_pulses(self) -> int:
        """Atomically read the number of pulses captured since last call."""

        with self._pulse_lock:
            return self._pulse_count

    def _rotor_rpm_from_frequency(self, frequency_hz: float) -> float:
        """Convert a pulse frequency into auger RPM using calibration constants."""

        motor_rpm = frequency_hz * MOTOR_RPM_PER_HZ
        pulley_ratio = self.rotor_pulley_diameter / self.motor_pulley_diameter
        return (motor_rpm / pulley_ratio) * ROTOR_CORRECTION

    def _control_loop(self) -> None:
        """Background thread: sample feedback, compute RPM, honour timers."""

        try:
            while not self._stop_event.is_set():
                now = time.time()
                cycle_expired = self._cycle_end_time is not None and now >= self._cycle_end_time
                if cycle_expired and self._cooldown_deadline is None:
                    self._cooldown_deadline = now + COOLDOWN_WINDOW_SEC
                    if self._current_percent > 0:
                        self._apply_pwm(0)
                        self._target_percent = 0.0
                if self._cooldown_deadline is not None and now >= self._cooldown_deadline:
                    break
                if self._interlock_open():
                    self.was_interrupted = True
                    break

                # Capture pulse counter at the start of the sampling window.
                with self._pulse_lock:
                    start_pulses = self._pulse_count
                if self._stop_event.wait(self.sample_time):
                    break

                # Capture pulse counter at the end of the sampling window.
                with self._pulse_lock:
                    end_pulses = self._pulse_count
                pulses = max(0, end_pulses - start_pulses)
                frequency = pulses / self.sample_time
                rotor_rpm = self._rotor_rpm_from_frequency(frequency)
                if pulses == 0:
                    self._update_rotor_rpm(0.0, EMA_ALPHA_TIMEOUT)
                else:
                    self._update_rotor_rpm(rotor_rpm, EMA_ALPHA_WINDOW)

            self._apply_pwm(0)
        finally:
            self._disarm_motor()
            self._control_thread = None
            if self.was_interrupted:
                self._stop_event.set()

    def _update_rotor_rpm(self, rotor_rpm: float, alpha: float) -> None:
        """Blend a new rotor RPM sample into the smoothed reading."""

        rotor_rpm = max(0.0, rotor_rpm)
        alpha = max(0.0, min(1.0, alpha))
        self._raw_rotor_rpm = rotor_rpm
        self._filtered_rotor_rpm = (
            alpha * rotor_rpm + (1.0 - alpha) * self._filtered_rotor_rpm
        )
        self._last_rotor_rpm = self._filtered_rotor_rpm

    def _set_watchdog(self, enable: bool) -> None:
        """Enable or disable the pigpio watchdog for the tach input."""

        if self.pi is None or not self.pi.connected:
            return
        timeout = WATCHDOG_TIMEOUT_MS if enable else 0
        try:
            self.pi.set_watchdog(self.pins.speed_feedback, timeout)
            self._watchdog_armed = enable
        except pigpio.error:
            self._watchdog_armed = False


__all__ = ["RPMControlledMotor", "MotorPins"]
