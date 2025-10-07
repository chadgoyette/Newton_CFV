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

import csv
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor, QFont
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


class RPMGraph(QWidget):
    """Custom widget that displays a real-time line graph of RPM values."""

    def __init__(self, max_points: int = 150, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.max_points = max_points
        self.rpm_data: list[float] = []
        self.setFixedHeight(120)
        self.setStyleSheet("background-color: #1a1a1a; border: 1px solid #444;")

    def add_data_point(self, rpm: float) -> None:
        """Add a new RPM value to the graph."""
        self.rpm_data.append(rpm)
        if len(self.rpm_data) > self.max_points:
            self.rpm_data.pop(0)
        self.update()  # Trigger repaint

    def clear_data(self) -> None:
        """Clear all data points from the graph."""
        self.rpm_data.clear()
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        """Draw the RPM graph using QPainter."""
        if not self.rpm_data:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Get dimensions for drawing
        width = self.width()
        height = self.height()
        margin_left = 50
        margin_right = 20
        margin_top = 20
        margin_bottom = 30
        graph_width = width - margin_left - margin_right
        graph_height = height - margin_top - margin_bottom

        # Calculate scale
        max_rpm = max(self.rpm_data) if self.rpm_data else 100
        max_rpm = max(max_rpm, 50)  # Minimum scale of 50 RPM
        max_rpm = ((max_rpm // 50) + 1) * 50  # Round up to nearest 50

        # Draw grid lines
        painter.setPen(QPen(QColor("#333333"), 1))
        num_horizontal_lines = 5
        for i in range(num_horizontal_lines + 1):
            y = margin_top + (graph_height * i / num_horizontal_lines)
            painter.drawLine(int(margin_left), int(y), int(width - margin_right), int(y))

        # Draw Y-axis labels
        painter.setPen(QPen(QColor("#888888"), 1))
        painter.setFont(QFont("Arial", 10))
        for i in range(num_horizontal_lines + 1):
            y = margin_top + (graph_height * i / num_horizontal_lines)
            rpm_value = max_rpm * (1 - i / num_horizontal_lines)
            painter.drawText(5, int(y + 5), f"{int(rpm_value)}")

        # Draw the RPM line
        if len(self.rpm_data) > 1:
            painter.setPen(QPen(QColor("#4caf50"), 2))
            x_step = graph_width / (self.max_points - 1)

            for i in range(len(self.rpm_data) - 1):
                x1 = margin_left + (i * x_step)
                x2 = margin_left + ((i + 1) * x_step)

                # Invert Y axis (higher RPM = higher on screen)
                y1 = margin_top + graph_height - (self.rpm_data[i] / max_rpm * graph_height)
                y2 = margin_top + graph_height - (self.rpm_data[i + 1] / max_rpm * graph_height)

                painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Draw axis labels
        painter.setPen(QPen(QColor("#aaaaaa"), 1))
        painter.setFont(QFont("Arial", 11, QFont.Bold))
        painter.drawText(margin_left, height - 5, "Time")
        painter.save()
        painter.translate(15, height // 2)
        painter.rotate(-90)
        painter.drawText(-30, 0, "RPM")
        painter.restore()


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
            sample_time=0.05,  # 50ms sampling for stable, accurate readings
            bypass_interlock=True,  # TODO: revert to False when safety switch is wired
        )

        # CSV logging setup
        self.logs_dir = Path(__file__).parent.parent / "logs"
        self.logs_dir.mkdir(exist_ok=True)
        self.log_file: Optional[object] = None
        self.log_writer: Optional[csv.writer] = None
        self.log_start_time: Optional[datetime] = None

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

        # Real-time RPM graph (300 points at 50ms = 15 seconds of history)
        self.rpm_graph = RPMGraph(max_points=300)

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
        main_layout.setSpacing(15)
        main_layout.addWidget(self.status_label)
        main_layout.addLayout(controls_layout)
        main_layout.addWidget(self.actual_rpm_label)
        main_layout.addWidget(self.rpm_graph)
        main_layout.addWidget(self.remaining_label)
        main_layout.addLayout(button_layout)
        root_layout = QHBoxLayout()
        root_layout.setContentsMargins(40, 40, 40, 40)
        root_layout.setSpacing(40)
        root_layout.addWidget(self.io_panel, stretch=0)
        root_layout.addLayout(main_layout, stretch=1)

        self.setLayout(root_layout)

        self.poll_timer = QTimer()
        self.poll_timer.setInterval(50)  # 50ms to match motor sample_time (20 Hz)
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
        self.rpm_graph.clear_data()  # Clear previous run's data
        
        # Create CSV log file for this run
        self.log_start_time = datetime.now()
        timestamp = self.log_start_time.strftime("%Y%m%d_%H%M%S")
        duration_str = f"{duration:.1f}s" if duration > 0 else "manual"
        filename = f"{timestamp}_{target_percent}pct_{duration_str}.csv"
        log_path = self.logs_dir / filename
        
        self.log_file = open(log_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["Timestamp", "Elapsed_Time_s", "Target_Percent", "Actual_RPM"])
        
        self.poll_timer.start()

    def stop_cycle(self) -> None:
        self.motor.stop_cycle()
        self.poll_timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.status_label.setText("Motor idle")
        self._close_log_file()
        self.update_status()

    def update_status(self) -> None:
        snapshot = self.motor.io_snapshot()
        self._refresh_io_indicators(snapshot)

        # Snapshot telemetry from the motor controller.
        rpm = self.motor.last_rotor_rpm
        self.actual_rpm_label.setText(f"Auger RPM: {rpm:5.1f}")
        
        # Add data point to the graph
        self.rpm_graph.add_data_point(rpm)
        
        # Log RPM data to CSV
        if self.log_writer is not None and self.log_start_time is not None:
            current_time = datetime.now()
            elapsed = (current_time - self.log_start_time).total_seconds()
            timestamp_str = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.log_writer.writerow([timestamp_str, f"{elapsed:.3f}", self.speed_spin.value(), f"{rpm:.2f}"])

        remaining = self._format_remaining_time(self.motor.remaining_cycle_time)
        self.remaining_label.setText(f"Time Remaining: {remaining}")

        if not self.motor.is_running:
            self.poll_timer.stop()
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self._close_log_file()
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

    def _close_log_file(self) -> None:
        """Close the current log file if one is open."""
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass  # Ignore errors when closing
            finally:
                self.log_file = None
                self.log_writer = None
                self.log_start_time = None

    # ------------------------------------------------------------------
    # Qt lifecycle
    # ------------------------------------------------------------------
    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.poll_timer.stop()
        self._close_log_file()
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
