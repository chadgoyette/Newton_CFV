#!/usr/bin/env python3
"""Simplified touchscreen UI for the Newton ice shaver controller.

The layout is intentionally compact to work well on the 5" touchscreen while
keeping the code legible by dividing responsibilities across the following
sections:

* Widget construction inside ``__init__`` builds the percentage selector and
  supporting controls.
* ``start_cycle`` / ``stop_cycle`` manage interaction with the motor driver.
* ``update_status`` polls the driver and refreshes the display.
* ``closeEvent`` ensures hardware resources get cleaned up when the window
  closes.
"""

import sys
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QSpinBox,
    QDoubleSpinBox,
    QMessageBox,
)

from .motor_rpm import MotorPins, RPMControlledMotor


class IOIndicator(QWidget):
    """Small LED-style indicator used to display GPIO states."""

    def __init__(self, label: str, *, is_pwm: bool = False, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        self.is_pwm = is_pwm

        self.light = QLabel()
        self.light.setFixedSize(18, 18)
        self.light.setStyleSheet(self._style(False))

        self.text_label = QLabel(label)
        self.text_label.setStyleSheet("font-size: 16px;")

        self.value_label: Optional[QLabel] = None
        if self.is_pwm:
            self.value_label = QLabel("0.0%")
            self.value_label.setStyleSheet("font-size: 14px; color: #aaaaaa;")

        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self.light, alignment=Qt.AlignLeft)
        layout.addWidget(self.text_label, alignment=Qt.AlignLeft)
        if self.value_label is not None:
            layout.addWidget(self.value_label, alignment=Qt.AlignLeft)
        layout.addStretch(1)
        self.setLayout(layout)

    @staticmethod
    def _style(active: bool) -> str:
        color = "#4caf50" if active else "#333333"
        border = "#77ff77" if active else "#111111"
        return (
            "background-color: {color};"
            "border: 1px solid {border};"
            "border-radius: 9px;"
        ).format(color=color, border=border)

    def set_state(self, active: bool, *, percentage: Optional[float] = None) -> None:
        self.light.setStyleSheet(self._style(active))
        if self.value_label is not None:
            if percentage is None:
                self.value_label.setText("--")
            else:
                self.value_label.setText(f"{percentage:4.1f}%")


class SimpleIceShaverUI(QWidget):
    """Minimal UI with RPM & cycle time controls for the ice shaver."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Newton Ice Shaver RPM Control")
        self.setFixedSize(800, 480)
        self.setStyleSheet("background-color: black; color: white;")

        # Motor driver wrapper handles closed-loop RPM control and interlock.
        self.motor = RPMControlledMotor(
            MotorPins(pwm=5, speed_feedback=16, enable=20, brake=19),
            bypass_interlock=True,  # TODO: revert to False when safety switch is wired
        )

        # Build the GPIO indicator column before composing the main UI.
        self.io_indicators: dict[str, IOIndicator] = {}
        io_layout = QVBoxLayout()
        io_layout.setContentsMargins(0, 0, 0, 0)
        io_layout.setSpacing(12)

        io_title = QLabel("GPIO Status")
        io_title.setStyleSheet("font-size: 18px; font-weight: bold;")
        io_layout.addWidget(io_title)

        for key, label, is_pwm in [
            ("pwm", "GPIO5 PWM", True),
            ("enable", "GPIO20 EN", False),
            ("brake", "GPIO19 BRK", False),
            ("direction", "GPIO12 DIR", False),
            ("speed_feedback", "GPIO16 SPD", False),
            ("interlock", "GPIO25 LID", False),
        ]:
            indicator = IOIndicator(label, is_pwm=is_pwm)
            self.io_indicators[key] = indicator
            io_layout.addWidget(indicator)

        io_layout.addStretch(1)
        self.io_panel = QWidget()
        self.io_panel.setLayout(io_layout)
        self.io_panel.setFixedWidth(200)

        self.status_label = QLabel("Motor idle")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 22px; font-weight: bold;")

        # Motor speed selector (0-100%) forwarded directly to the driver.
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(0, 100)
        self.speed_spin.setSingleStep(1)
        self.speed_spin.setSuffix(" %")
        self.speed_spin.setFixedHeight(80)
        self.speed_spin.setFixedWidth(200)
        # Make the touch targets larger by widening the arrow buttons.
        self.speed_spin.setStyleSheet(
            """
            QSpinBox {
                font-size: 42px;
                background-color: #333;
            }
            QSpinBox::up-button {
                width: 48px;
            }
            QSpinBox::down-button {
                width: 48px;
            }
            """
        )

        # Cycle duration selector (0-20 seconds in 0.5 second steps).
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.0, 20.0)
        self.duration_spin.setSingleStep(0.5)
        self.duration_spin.setDecimals(1)
        self.duration_spin.setSuffix(" s")
        self.duration_spin.setFixedHeight(80)
        self.duration_spin.setFixedWidth(200)
        self.duration_spin.setStyleSheet(
            """
            QDoubleSpinBox {
                font-size: 42px;
                background-color: #333;
            }
            QDoubleSpinBox::up-button {
                width: 48px;
            }
            QDoubleSpinBox::down-button {
                width: 48px;
            }
            """
        )

        self.actual_rpm_label = QLabel("Auger RPM: 0")
        self.actual_rpm_label.setAlignment(Qt.AlignCenter)
        self.actual_rpm_label.setStyleSheet("font-size: 36px; font-weight: bold;")

        self.remaining_label = QLabel("Time Remaining: --")
        self.remaining_label.setAlignment(Qt.AlignCenter)
        self.remaining_label.setStyleSheet("font-size: 24px;")

        self.start_button = QPushButton("Start Cycle")
        self.start_button.setFixedSize(200, 120)
        self.start_button.setStyleSheet("font-size: 24px; background-color: green;")
        self.start_button.clicked.connect(self.start_cycle)

        self.stop_button = QPushButton("Stop")
        self.stop_button.setFixedSize(200, 120)
        self.stop_button.setStyleSheet("font-size: 24px; background-color: red;")
        self.stop_button.clicked.connect(self.stop_cycle)
        self.stop_button.setEnabled(False)

        controls_layout = QHBoxLayout()
        controls_layout.addStretch(1)
        controls_layout.addWidget(self.speed_spin)
        controls_layout.addSpacing(20)
        controls_layout.addWidget(self.duration_spin)
        controls_layout.addStretch(1)

        button_layout = QHBoxLayout()
        button_layout.addStretch(1)
        button_layout.addWidget(self.start_button)
        button_layout.addSpacing(20)
        button_layout.addWidget(self.stop_button)
        button_layout.addStretch(1)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(30)
        main_layout.addWidget(self.status_label)
        main_layout.addLayout(controls_layout)
        main_layout.addWidget(self.actual_rpm_label)
        main_layout.addWidget(self.remaining_label)
        main_layout.addLayout(button_layout)
        root_layout = QHBoxLayout()
        root_layout.setContentsMargins(40, 40, 40, 40)
        root_layout.setSpacing(40)
        root_layout.addWidget(self.io_panel, stretch=0)
        root_layout.addLayout(main_layout, stretch=1)

        self.setLayout(root_layout)

        self.poll_timer = QTimer()
        self.poll_timer.setInterval(200)
        self.poll_timer.timeout.connect(self.update_status)

        # Seed the indicator column with the current GPIO states.
        self._refresh_io_indicators(self.motor.io_snapshot())

    # ------------------------------------------------------------------
    # UI actions
    # ------------------------------------------------------------------
    def start_cycle(self) -> None:
        target_percent = self.speed_spin.value()
        duration = self.duration_spin.value()

        if target_percent <= 0:
            QMessageBox.warning(self, "Invalid Speed", "Set a percentage above zero to start.")
            return

        try:
            # Ask the motor module to command the requested duty cycle.
            self.motor.start_cycle(target_percent, duration)
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid Input", str(exc))
            return
        except RuntimeError as exc:
            QMessageBox.critical(self, "Motor Error", str(exc))
            return

        self.status_label.setText(
            f"Running: {target_percent}% for {'manual stop' if duration == 0 else f'{duration:.1f}s'}"
        )
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.poll_timer.start()

    def stop_cycle(self) -> None:
        self.motor.stop_cycle()
        self.poll_timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("Motor idle")
        self.update_status()

    def update_status(self) -> None:
        snapshot = self.motor.io_snapshot()
        self._refresh_io_indicators(snapshot)

        # Snapshot telemetry from the motor controller.
        rpm = self.motor.last_rotor_rpm
        self.actual_rpm_label.setText(f"Auger RPM: {rpm:5.1f}")

        remaining = self._format_remaining_time(self.motor.remaining_cycle_time)
        self.remaining_label.setText(f"Time Remaining: {remaining}")

        if not self.motor.is_running:
            self.poll_timer.stop()
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            if self.motor.was_interrupted:
                # Surface interlock trips prominently so operators notice.
                QMessageBox.warning(
                    self,
                    "Safety Interlock",
                    "Cover opened during the cycle. Motor stopped for safety.",
                )
                self.motor.was_interrupted = False
            self.status_label.setText("Motor idle")

    def _refresh_io_indicators(self, snapshot: dict[str, float | int | bool]) -> None:
        if not snapshot:
            return

        pwm_indicator = self.io_indicators.get("pwm")
        if pwm_indicator is not None:
            pwm_raw = snapshot.get("pwm_raw", 0)
            pwm_indicator.set_state(pwm_raw > 0, percentage=snapshot.get("pwm_percent"))

        def digital_active(value: object) -> bool:
            if isinstance(value, bool):
                return value
            try:
                return int(value) == 1
            except (TypeError, ValueError):
                return False

        for key in ("enable", "brake", "direction", "speed_feedback", "interlock"):
            indicator = self.io_indicators.get(key)
            if indicator is None:
                continue
            indicator.set_state(digital_active(snapshot.get(key)))

    def _format_remaining_time(self, remaining: Optional[float]) -> str:
        if remaining is None:
            return "--"
        minutes = int(remaining // 60)
        seconds = int(remaining % 60)
        tenths = int((remaining - int(remaining)) * 10)
        return f"{minutes}:{seconds:02d}.{tenths}"

    # ------------------------------------------------------------------
    # Qt lifecycle
    # ------------------------------------------------------------------
    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.poll_timer.stop()
        try:
            self.motor.shutdown()
        finally:
            super().closeEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    ui = SimpleIceShaverUI()
    ui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
