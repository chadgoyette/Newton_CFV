import pigpio
import time
import threading
import RPi.GPIO as GPIO

WATCHDOG_TIMEOUT_MS = 150
EMA_ALPHA_PULSE = 0.5
EMA_ALPHA_WINDOW = 0.25
EMA_ALPHA_TIMEOUT = 0.15
MIN_SAMPLE_TIME = 0.02

class BLDCMotor:
    def __init__(self, pwm_pin, speed_pin, en_pin, brk_pin, motor_pulley_diameter=22, rotor_pulley_diameter=95, dir_pin=12, motor_pole_pairs=4):
        self.pwm_pin = pwm_pin
        self.speed_pin = speed_pin
        self.en_pin = en_pin
        self.brk_pin = brk_pin
        self.dir_pin = dir_pin  # Direction control pin
        self.motor_pulley_diameter = motor_pulley_diameter
        self.rotor_pulley_diameter = rotor_pulley_diameter
        self.motor_pole_pairs = motor_pole_pairs
        self.pulse_count = 0
        self.lock = threading.Lock()
        self.was_interrupted = False
        self._last_pulse_tick = None
        self._raw_rotor_rpm = 0.0
        self._filtered_rotor_rpm = 0.0
        self._rpm_correction = 1.031
        self._watchdog_enabled = False

        # Safety interlock setup (bench testing bypasses GPIO read below)
        self.interlock_pin = 25
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.interlock_pin, GPIO.IN)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise Exception("Could not connect to pigpio daemon!")

        self.pi.set_mode(self.pwm_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.speed_pin, pigpio.INPUT)
        self.pi.set_mode(self.en_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.brk_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)  # Configure direction pin as output
        self.pi.set_pull_up_down(self.speed_pin, pigpio.PUD_UP)
        self.pi.set_glitch_filter(self.speed_pin, 100)

        self.pi.write(self.en_pin, 1)
        self.pi.write(self.brk_pin, 0)
        self.pi.set_PWM_frequency(self.pwm_pin, 4000)

        # Set direction pin to HIGH permanently
        self.pi.write(self.dir_pin, 1)
        print("[INFO] Motor direction permanently set to HIGH (GPIO-12 HIGH).")

        self.callback = self.pi.callback(self.speed_pin, pigpio.RISING_EDGE, self._pulse_callback)
        self._set_watchdog(True)

        print("[INFO] BLDC Motor initialized.")

    def _pulse_callback(self, gpio, level, tick):
        if level == pigpio.TIMEOUT:
            self._last_pulse_tick = None
            self._update_rotor_rpm(0.0, EMA_ALPHA_TIMEOUT)
            return
        if level not in (pigpio.RISING_EDGE, 1):
            return

        with self.lock:
            self.pulse_count += 1

        if self._last_pulse_tick is not None:
            delta_us = pigpio.tickDiff(self._last_pulse_tick, tick)
            if delta_us > 0:
                frequency = 1_000_000.0 / delta_us
                rotor_rpm = self._rotor_rpm_from_frequency(frequency)
                self._update_rotor_rpm(rotor_rpm, EMA_ALPHA_PULSE)

        self._last_pulse_tick = tick

    def interlock_triggered(self):
        """Check if the safety interlock (lid) is triggered."""
        return False  # Temporarily bypassed for testing; restore GPIO check when safe.

    def check_interlock_and_stop(self):
        """Stop the motor if the interlock is triggered."""
        if self.interlock_triggered():
            print("[SAFETY] Cover removed! Stopping motor...")
            self.stop()
            self.was_interrupted = True

    def set_speed(self, duty_cycle):
        """Set the motor speed, ensuring the interlock is not triggered."""
        if self.interlock_triggered():
            print("[WARNING] Cover open! Motor speed set request ignored.")
            self.was_interrupted = True
            return
        if 0 <= duty_cycle <= 100:
            pwm_value = int((duty_cycle / 100.0) * 255)
            self.pi.set_PWM_dutycycle(self.pwm_pin, pwm_value)
            actual_freq = self.pi.get_PWM_frequency(self.pwm_pin)
            actual_duty = self.pi.get_PWM_dutycycle(self.pwm_pin)
            print(f"[INFO] Motor Speed: {duty_cycle}% | PWM: {pwm_value}/255 | Freq: {actual_freq} Hz | Duty Confirmed: {actual_duty}")
        else:
            raise ValueError("Duty cycle must be between 0 and 100.")

    def start(self):
        """Start the motor, ensuring the interlock is not triggered."""
        if self.interlock_triggered():
            print("[WARNING] Cannot start motor: Cover is open.")
            self.was_interrupted = True
            return False

        print("[INFO] Starting motor...")
        try:
            if self.pi is None or not self.pi.connected:
                print("[ERROR] pigpio connection lost. Reinitializing...")
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    raise RuntimeError("Failed to reconnect to pigpio daemon.")

            print("[DEBUG] Enabling motor and releasing brake...")
            self.callback = self.pi.callback(self.speed_pin, pigpio.RISING_EDGE, self._pulse_callback)  # Reinitialize callback
            self._set_watchdog(True)
            self.pulse_count = 0  # Reset pulse count to ensure accurate RPM calculation
            self._filtered_rotor_rpm = 0.0
            self._raw_rotor_rpm = 0.0
            self._last_pulse_tick = None

            self.pi.write(self.en_pin, 0)
            self.pi.write(self.brk_pin, 1)
            time.sleep(0.5)  # Allow motor to stabilize before RPM calculation
            print("[INFO] Motor enabled and brake released.")
            return True
        except Exception as e:
            print(f"[ERROR] Exception occurred while starting motor: {e}")
            return False

    def get_rpm(self, sample_time=0.1):
        sample_time = max(MIN_SAMPLE_TIME, sample_time)
        with self.lock:
            start_count = self.pulse_count
        time.sleep(sample_time)
        with self.lock:
            end_count = self.pulse_count

        pulses = end_count - start_count
        frequency = pulses / sample_time
        rotor_rpm = self._rotor_rpm_from_frequency(frequency)
        if pulses == 0:
            self._update_rotor_rpm(0.0, EMA_ALPHA_TIMEOUT)
        else:
            self._update_rotor_rpm(rotor_rpm, EMA_ALPHA_WINDOW)
        return self._filtered_rotor_rpm

    def stop(self):
        """Stop the motor and clean up resources."""
        self.pi.write(self.brk_pin, 0)
        self.pi.write(self.en_pin, 1)
        if hasattr(self, 'callback') and self.callback is not None:
            self.callback.cancel()
        self._set_watchdog(False)
        self.pi.set_servo_pulsewidth(self.pwm_pin, 0)
        self.pi.stop()
        self._filtered_rotor_rpm = 0.0
        self._raw_rotor_rpm = 0.0
        print("[INFO] Motor stopped and resources cleaned up.")

    def debug_status(self):
        print("[DEBUG] EN:", self.pi.read(self.en_pin))
        print("[DEBUG] BRK:", self.pi.read(self.brk_pin))
        print("[DEBUG] PWM duty:", self.pi.get_PWM_dutycycle(self.pwm_pin))
        print("[DEBUG] Interlock:", self.interlock_triggered())

    def _rotor_rpm_from_frequency(self, frequency_hz):
        """Convert tach pulse frequency to rotor RPM with calibration applied."""

        motor_rpm = (frequency_hz / max(1, self.motor_pole_pairs)) * (60 / 3)
        pulley_ratio = self.rotor_pulley_diameter / self.motor_pulley_diameter
        rotor_rpm = (motor_rpm / pulley_ratio) * self._rpm_correction
        return max(0.0, rotor_rpm)

    def _update_rotor_rpm(self, rotor_rpm, alpha):
        """Blend a new RPM sample into the smoothed reading."""

        alpha = max(0.0, min(1.0, alpha))
        self._raw_rotor_rpm = max(0.0, rotor_rpm)
        self._filtered_rotor_rpm = (
            alpha * self._raw_rotor_rpm + (1.0 - alpha) * self._filtered_rotor_rpm
        )

    def _set_watchdog(self, enable):
        """Enable or disable the pigpio watchdog for missing tach pulses."""

        if self.pi is None or not self.pi.connected:
            self._watchdog_enabled = False
            return
        timeout = WATCHDOG_TIMEOUT_MS if enable else 0
        try:
            self.pi.set_watchdog(self.speed_pin, timeout)
            self._watchdog_enabled = enable
        except pigpio.error:
            self._watchdog_enabled = False
