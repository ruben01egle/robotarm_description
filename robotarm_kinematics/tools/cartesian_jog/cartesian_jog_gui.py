#!/usr/bin/env python3
"""
Six sliders for the Cartesian velocity of the tool, published as a Twist on cmd_vel.

Companion of the cartesian_jog node (like joint_state_publisher_gui is for joint jogging).
The twist is expressed in the BASE frame: x, y, z in m/s and the rotation rates about the base
x, y, z axes in rad/s.

The sliders spring back to zero when released, so letting go stops the tool. "Keep values on
release" leaves them where they are (the tool then keeps moving until they are zeroed).
Messages are only published while a slider is away from zero, plus one final zero twist, so
other publishers on cmd_vel are not disturbed while everything is at rest.

Parameters: max_linear [m/s] (0.1), max_angular [rad/s] (0.5), rate [Hz] (20).
"""

import signal
import sys

from geometry_msgs.msg import Twist
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication, QCheckBox, QGridLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)
import rclpy

SLIDER_STEPS = 1000  # slider range is [-SLIDER_STEPS, SLIDER_STEPS]

AXES = (
    ('x', 'm/s', 'linear'),
    ('y', 'm/s', 'linear'),
    ('z', 'm/s', 'linear'),
    ('wx', 'rad/s', 'angular'),
    ('wy', 'rad/s', 'angular'),
    ('wz', 'rad/s', 'angular'),
)


class JogWindow(QWidget):
    """Window with one slider per Cartesian degree of freedom."""

    def __init__(self, max_linear, max_angular, publish, rate=20.0):
        """Create the window; publish(values) is called with the 6 twist values at `rate`."""
        super().__init__()
        self._publish = publish
        self._scales = [max_linear if kind == 'linear' else max_angular for _, _, kind in AXES]
        self._was_active = False

        self.setWindowTitle('Cartesian jog (base frame)')
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('Tool velocity in the base frame, published on cmd_vel'))

        grid = QGridLayout()
        layout.addLayout(grid)
        self.sliders = []
        self._value_labels = []
        for row, (name, unit, _) in enumerate(AXES):
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-SLIDER_STEPS, SLIDER_STEPS)
            slider.setValue(0)
            slider.setMinimumWidth(320)
            slider.valueChanged.connect(self._update_labels)
            slider.sliderReleased.connect(self._on_released)
            value_label = QLabel()
            value_label.setMinimumWidth(90)
            grid.addWidget(QLabel(name), row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(value_label, row, 2)
            self.sliders.append(slider)
            self._value_labels.append(value_label)

        self._keep = QCheckBox('Keep values on release')
        layout.addWidget(self._keep)
        zero_button = QPushButton('Zero all')
        zero_button.clicked.connect(self.zero)
        layout.addWidget(zero_button)

        self._update_labels()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.tick)
        self._timer.start(int(round(1000.0 / rate)))

    def values(self):
        """Return the current twist: (x, y, z) in m/s and (wx, wy, wz) in rad/s."""
        return tuple(
            slider.value() / SLIDER_STEPS * scale
            for slider, scale in zip(self.sliders, self._scales))

    def zero(self):
        """Put all sliders back to zero."""
        for slider in self.sliders:
            slider.setValue(0)

    def tick(self):
        """Publish the twist while it is not zero, plus once when it becomes zero."""
        values = self.values()
        active = any(v != 0.0 for v in values)
        if active or self._was_active:
            self._publish(values)
        self._was_active = active

    def _on_released(self):
        if not self._keep.isChecked():
            # every slider springs back on its own release, the others stay where they are
            sender = self.sender()
            if sender is not None:
                sender.setValue(0)

    def _update_labels(self):
        for label, value, (_, unit, _) in zip(self._value_labels, self.values(), AXES):
            label.setText(f'{value:+.3f} {unit}')


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('cartesian_jog_gui')
    max_linear = node.declare_parameter('max_linear', 0.1).value
    max_angular = node.declare_parameter('max_angular', 0.5).value
    rate = node.declare_parameter('rate', 20.0).value
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    def publish(values):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = (float(v) for v in values[:3])
        msg.angular.x, msg.angular.y, msg.angular.z = (float(v) for v in values[3:])
        publisher.publish(msg)

    app = QApplication(sys.argv)
    window = JogWindow(max_linear, max_angular, publish, rate)
    window.show()

    # the node has no callbacks of its own, but spinning it keeps its parameter services answering
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(20)

    # Ctrl+C / SIGINT from the launch system: leave the event loop, so the process exits cleanly
    # (the spin timer above wakes the interpreter often enough to run this handler)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    result = app.exec_()
    node.destroy_node()
    rclpy.try_shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
