#!/usr/bin/env python3
"""Qt control panel for the physical manipulator.

The GUI publishes the same high-level topics as the joystick controls. The
``arm_commander`` node remains responsible for IK, command clamping, and the
50 Hz command stream sent to the ESP32.
"""

import signal
import sys
import time
from math import cos, degrees, radians, sin

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32, Float32MultiArray

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from manipulator_control import arm_ik_2d as ik


BASE_SPEED_MIN = 0.1
BASE_SPEED_MAX = 1.0
TARGET_STEP = 0.005
# Spinbox display resolution (3 decimals); the start target is snapped to it.
TARGET_STEP_MM = 0.001
REACH_MIN = 0.11
REACH_MAX = 0.33
HEIGHT_MIN = 0.00
HEIGHT_MAX = 0.15


class ArmControlNode(Node):
    """ROS interface kept separate from the Qt widgets."""

    def __init__(self):
        super().__init__('arm_control_gui')
        self.base_pub = self.create_publisher(Float32, '/base_cmd', 10)
        self.target_pub = self.create_publisher(Point, '/target_pose', 10)
        self.gripper_pub = self.create_publisher(Float32, '/gripper_cmd', 10)
        self.create_subscription(
            Float32MultiArray, '/arm_command', self._command_cb, 10)
        self.create_subscription(
            Float32, '/shoulder_feedback', self._feedback_cb, 10)

        self.last_command = None
        self.last_command_time = None
        self.last_feedback = None
        self.last_feedback_time = None

    def _command_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self.last_command = tuple(msg.data[:5])
            self.last_command_time = time.monotonic()

    def _feedback_cb(self, msg: Float32):
        self.last_feedback = msg.data
        self.last_feedback_time = time.monotonic()

    def send_base(self, velocity: float):
        msg = Float32()
        msg.data = float(max(-1.0, min(1.0, velocity)))
        self.base_pub.publish(msg)

    def send_target(self, x: float, y: float, z: float):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.target_pub.publish(msg)

    def send_gripper(self, position: float):
        msg = Float32()
        msg.data = float(max(0.0, min(1.0, position)))
        self.gripper_pub.publish(msg)


class ArmControlWindow(QMainWindow):
    def __init__(self, node: ArmControlNode):
        super().__init__()
        self.node = node
        self._base_is_moving = False

        self.setWindowTitle('Manipulator Control')
        self.setMinimumWidth(560)
        self.setStyleSheet(self._style_sheet())

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(14)
        self.setCentralWidget(central)

        title = QLabel('Manipulator Control')
        title.setObjectName('title')
        subtitle = QLabel(
            'Cylindrical targets, base velocity, and gripper position')
        subtitle.setObjectName('subtitle')
        root.addWidget(title)
        root.addWidget(subtitle)

        self.status_label = QLabel('Waiting for arm_commander…')
        self.status_label.setObjectName('statusWaiting')
        root.addWidget(self.status_label)

        root.addWidget(self._build_base_group())
        root.addWidget(self._build_target_group())
        root.addWidget(self._build_gripper_group())

        self.feedback_label = QLabel('No /arm_command received yet.')
        self.feedback_label.setObjectName('feedback')
        self.feedback_label.setWordWrap(True)
        self.feedback_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root.addWidget(self.feedback_label)

        self.feedback_pot_label = QLabel('Potentiometer: no /shoulder_feedback yet.')
        self.feedback_pot_label.setObjectName('feedback')
        root.addWidget(self.feedback_pot_label)

        self.event_label = QLabel('Ready')
        self.event_label.setObjectName('event')
        root.addWidget(self.event_label)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(250)

    @staticmethod
    def _icon_button(symbol, tooltip, object_name=None):
        """Square symbol button. The tooltip carries the wording it replaces."""
        button = QPushButton(symbol)
        button.setToolTip(tooltip)
        button.setObjectName(object_name or 'iconButton')
        return button

    def _build_base_group(self):
        group = QGroupBox('Base rotation  ⟳')
        layout = QVBoxLayout(group)

        speed_row = QHBoxLayout()
        speed_icon = QLabel('»')
        speed_icon.setObjectName('fieldIcon')
        speed_icon.setToolTip('Rotation speed')
        speed_row.addWidget(speed_icon)
        self.base_speed = QSlider(Qt.Horizontal)
        self.base_speed.setRange(
            round(BASE_SPEED_MIN * 100), round(BASE_SPEED_MAX * 100))
        self.base_speed.setValue(50)
        self.base_speed.setToolTip('Rotation speed')
        self.base_speed.valueChanged.connect(self._update_speed_label)
        speed_row.addWidget(self.base_speed, 1)
        self.speed_label = QLabel('0.50')
        self.speed_label.setMinimumWidth(38)
        speed_row.addWidget(self.speed_label)
        layout.addLayout(speed_row)

        buttons = QHBoxLayout()
        # Spatial order: counter-clockwise on the left, clockwise on the right.
        left = self._icon_button('↺', 'Hold to rotate left (counter-clockwise)')
        stop = self._icon_button('■', 'Stop base', 'stopButton')
        right = self._icon_button('↻', 'Hold to rotate right (clockwise)')

        left.pressed.connect(lambda: self._start_base(1.0))
        left.released.connect(self._stop_base)
        right.pressed.connect(lambda: self._start_base(-1.0))
        right.released.connect(self._stop_base)
        stop.clicked.connect(self._stop_base)

        buttons.addWidget(left)
        buttons.addWidget(stop)
        buttons.addWidget(right)
        layout.addLayout(buttons)

        hint = QLabel('Hold ↺ / ↻ to spin — releasing stops the base.')
        hint.setObjectName('hint')
        layout.addWidget(hint)
        return group

    def _build_target_group(self):
        group = QGroupBox('End-effector target  ✛')
        layout = QVBoxLayout(group)

        start_reach, start_height = self._start_target()

        self.reach = self._distance_spinbox(
            REACH_MIN, REACH_MAX, start_reach)
        self.reach.setToolTip('Reach from base axis')
        self.height = self._distance_spinbox(
            HEIGHT_MIN, HEIGHT_MAX, start_height)
        self.height.setToolTip('Tip height above ground')
        self.azimuth = QDoubleSpinBox()
        self.azimuth.setDecimals(0)
        self.azimuth.setSingleStep(5.0)
        self.azimuth.setRange(-180.0, 180.0)
        self.azimuth.setValue(0.0)
        self.azimuth.setSuffix('°')
        self.azimuth.setKeyboardTracking(False)
        self.azimuth.setToolTip('Base azimuth (yaw) of the target')

        fields = QGridLayout()
        for row, (symbol, tip, widget) in enumerate((
            ('∠', 'Base azimuth (yaw)', self.azimuth),
            ('⟷', 'Reach from base axis', self.reach),
            ('↕', 'Height above ground', self.height),
        )):
            icon = QLabel(symbol)
            icon.setObjectName('fieldIcon')
            icon.setToolTip(tip)
            icon.setAlignment(Qt.AlignCenter)
            fields.addWidget(icon, row, 0)
            fields.addWidget(widget, row, 1)
        fields.setColumnStretch(1, 1)
        layout.addLayout(fields)

        # Jog pad laid out the way the arm moves: up/down = height,
        # left/right = reach, centre = return to the safe start pose.
        pad = QGridLayout()
        pad.setSpacing(6)
        raise_arm = self._icon_button('▲', f'Raise {TARGET_STEP * 1000:.0f} mm')
        lower = self._icon_button('▼', f'Lower {TARGET_STEP * 1000:.0f} mm')
        reach_less = self._icon_button(
            '◀', f'Reach in {TARGET_STEP * 1000:.0f} mm')
        reach_more = self._icon_button(
            '▶', f'Reach out {TARGET_STEP * 1000:.0f} mm')
        safe_start = self._icon_button('⌂', 'Go to safe start target')

        raise_arm.clicked.connect(lambda: self._jog_target(0.0, TARGET_STEP))
        lower.clicked.connect(lambda: self._jog_target(0.0, -TARGET_STEP))
        reach_less.clicked.connect(lambda: self._jog_target(-TARGET_STEP, 0.0))
        reach_more.clicked.connect(lambda: self._jog_target(TARGET_STEP, 0.0))
        safe_start.clicked.connect(self._send_safe_start)

        pad.addWidget(raise_arm, 0, 1)
        pad.addWidget(reach_less, 1, 0)
        pad.addWidget(safe_start, 1, 1)
        pad.addWidget(reach_more, 1, 2)
        pad.addWidget(lower, 2, 1)

        pad_row = QHBoxLayout()
        pad_row.addStretch(1)
        pad_row.addLayout(pad)
        pad_row.addStretch(1)
        layout.addLayout(pad_row)

        move = self._icon_button('➤  Move', 'Send the target to the arm',
                                 'primaryButton')
        layout.addWidget(move)
        move.clicked.connect(self._send_target)

        polar_hint = QLabel(
            '∠ azimuth · ⟷ reach · ↕ height. Simulation solves ∠ as J1; '
            'physical J1 stays manual without an encoder.')
        polar_hint.setObjectName('hint')
        polar_hint.setWordWrap(True)
        layout.addWidget(polar_hint)
        return group

    def _build_gripper_group(self):
        group = QGroupBox('Gripper  ⊑⊒')
        layout = QVBoxLayout(group)

        # Slider runs left (open) to right (closed), so the end labels show the
        # finger pair spread apart and pressed together.
        position_row = QHBoxLayout()
        open_icon = QLabel('◀   ▶')
        open_icon.setObjectName('fieldIcon')
        open_icon.setToolTip('Open')
        position_row.addWidget(open_icon)
        self.gripper = QSlider(Qt.Horizontal)
        self.gripper.setRange(0, 100)
        self.gripper.setValue(0)
        self.gripper.setToolTip('Gripper position: left open, right closed')
        self.gripper.valueChanged.connect(self._update_gripper_label)
        self.gripper.sliderReleased.connect(self._send_gripper)
        position_row.addWidget(self.gripper, 1)
        closed_icon = QLabel('▶ ◀')
        closed_icon.setObjectName('fieldIcon')
        closed_icon.setToolTip('Closed')
        position_row.addWidget(closed_icon)
        self.gripper_label = QLabel('0%')
        self.gripper_label.setMinimumWidth(38)
        position_row.addWidget(self.gripper_label)
        layout.addLayout(position_row)

        buttons = QHBoxLayout()
        open_button = self._icon_button('◀   ▶', 'Open fully')
        # '✔' measures ~82 ink px at 24 px vs '✓' at 53 (offscreen render
        # check), so it stays legible at button size.
        apply_button = self._icon_button(
            '✔', 'Send the slider position', 'primaryIconButton')
        close_button = self._icon_button('▶ ◀', 'Close fully')
        open_button.clicked.connect(lambda: self._set_gripper(0))
        apply_button.clicked.connect(self._send_gripper)
        close_button.clicked.connect(lambda: self._set_gripper(100))
        buttons.addWidget(open_button)
        buttons.addWidget(apply_button)
        buttons.addWidget(close_button)
        layout.addLayout(buttons)
        return group

    @staticmethod
    def _distance_spinbox(minimum, maximum, value):
        spin = QDoubleSpinBox()
        spin.setDecimals(3)
        spin.setSingleStep(TARGET_STEP)
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSuffix(' m')
        spin.setKeyboardTracking(False)
        return spin

    @staticmethod
    def _start_target():
        """Safe-start tip target, snapped to a value the spinboxes can hold.

        The raw FK of the start pose sits on the workspace boundary, so the
        3-decimal spinbox rounding can land outside it. Snapping keeps the
        displayed default sendable.
        """
        reach, height = ik.forward(
            ik.START_SHOULDER, ik.START_ELBOW, ik.START_WRIST)
        snapped = ik.nearest_feasible(reach, height, step=TARGET_STEP_MM)
        return snapped if snapped is not None else (reach, height)

    def _update_speed_label(self, value):
        self.speed_label.setText(f'{value / 100.0:.2f}')

    def _start_base(self, direction):
        velocity = direction * self.base_speed.value() / 100.0
        self.node.send_base(velocity)
        self._base_is_moving = True
        side = 'left' if direction > 0 else 'right'
        self._set_event(f'Base rotating {side} at {abs(velocity):.2f}')

    def _stop_base(self):
        self.node.send_base(0.0)
        self._base_is_moving = False
        self._set_event('Base stopped')

    def _jog_target(self, reach_delta, height_delta):
        self.reach.setValue(self.reach.value() + reach_delta)
        self.height.setValue(self.height.value() + height_delta)
        self._send_target()

    def _send_target(self):
        azimuth_deg = self.azimuth.value()
        reach = self.reach.value()
        height = self.height.value()
        azimuth = radians(azimuth_deg)
        x = reach * cos(azimuth)
        y = reach * sin(azimuth)
        if ik.solve_cartesian(x, y, height) is None:
            self._set_event(
                f'Target is outside the safe workspace: '
                f'r={reach:.3f} m, z={height:.3f} m',
                warning=True)
            return
        self.node.send_target(x, y, height)
        self._set_event(
            f'Polar target sent: azimuth {azimuth_deg:+.0f}°, '
            f'reach {reach:.3f} m, height {height:.3f} m')

    def _send_safe_start(self):
        reach, height = self._start_target()
        self.azimuth.setValue(0.0)
        self.reach.setValue(reach)
        self.height.setValue(height)
        self._send_target()

    def _update_gripper_label(self, value):
        self.gripper_label.setText(f'{value}%')

    def _set_gripper(self, value):
        self.gripper.setValue(value)
        self._send_gripper()

    def _send_gripper(self):
        position = self.gripper.value() / 100.0
        self.node.send_gripper(position)
        self._set_event(f'Gripper position sent: {position:.2f}')

    def _set_event(self, text, warning=False):
        self.event_label.setText(text)
        self.event_label.setObjectName('eventWarning' if warning else 'event')
        self.event_label.style().unpolish(self.event_label)
        self.event_label.style().polish(self.event_label)

    def _refresh_status(self):
        age = None
        if self.node.last_command_time is not None:
            age = time.monotonic() - self.node.last_command_time

        if age is not None and age < 1.0:
            self.status_label.setText('Connected — arm_commander is publishing')
            self.status_label.setObjectName('statusConnected')
            base, shoulder, elbow, wrist, gripper = self.node.last_command
            self.feedback_label.setText(
                f'Commanded: base {base:+.2f}  |  shoulder {shoulder:+.2f} rad  |  '
                f'elbow {elbow:+.2f} rad  |  wrist {wrist:+.2f} rad  |  '
                f'gripper {gripper:.2f}')
        else:
            self.status_label.setText('Waiting for /arm_command from arm_commander…')
            self.status_label.setObjectName('statusWaiting')

        fb_age = None
        if self.node.last_feedback_time is not None:
            fb_age = time.monotonic() - self.node.last_feedback_time
        if fb_age is not None and fb_age < 1.0:
            shoulder_deg = degrees(self.node.last_feedback)
            self.feedback_pot_label.setText(
                f'Potentiometer (shoulder): {shoulder_deg:.1f}° live')
        else:
            self.feedback_pot_label.setText('Potentiometer: no /shoulder_feedback yet.')

        # Qt does not automatically re-polish a widget after its object name changes.
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def closeEvent(self, event):
        self._stop_base()
        event.accept()

    @staticmethod
    def _style_sheet():
        return """
            QMainWindow, QWidget {
                background: #171b22;
                color: #e8edf2;
                font-size: 13px;
            }
            QLabel#title {
                font-size: 24px;
                font-weight: 700;
                color: #ffffff;
            }
            QLabel#subtitle, QLabel#hint {
                color: #9ba8b4;
            }
            QLabel#statusConnected {
                color: #67d391;
                font-weight: 600;
            }
            QLabel#statusWaiting {
                color: #f4c56a;
                font-weight: 600;
            }
            QLabel#feedback, QLabel#event, QLabel#eventWarning {
                background: #10141a;
                border: 1px solid #303844;
                border-radius: 6px;
                padding: 9px;
            }
            QLabel#eventWarning { color: #f4c56a; }
            QGroupBox {
                border: 1px solid #38414d;
                border-radius: 8px;
                margin-top: 10px;
                padding: 12px 8px 8px 8px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
            }
            QPushButton {
                background: #2d3642;
                border: 1px solid #465262;
                border-radius: 6px;
                min-height: 34px;
                padding: 2px 10px;
            }
            QPushButton:hover { background: #394555; }
            QPushButton:pressed { background: #222a34; }
            QPushButton#primaryButton {
                background: #2774c7;
                border-color: #3f8de0;
                font-weight: 600;
            }
            QPushButton#stopButton {
                background: #a6313c;
                border-color: #d64b59;
                font-weight: 700;
            }
            QPushButton#iconButton, QPushButton#stopButton {
                font-size: 20px;
                min-width: 62px;
                min-height: 46px;
                padding: 0;
            }
            QPushButton#primaryButton {
                font-size: 16px;
                min-height: 42px;
            }
            QPushButton#primaryIconButton {
                background: #2774c7;
                border: 1px solid #3f8de0;
                border-radius: 6px;
                font-size: 24px;
                font-weight: 700;
                min-width: 62px;
                min-height: 46px;
                padding: 0;
            }
            QPushButton#primaryIconButton:hover { background: #3184dc; }
            QPushButton#primaryIconButton:pressed { background: #1f5fa6; }
            QLabel#fieldIcon {
                font-size: 17px;
                color: #9ba8b4;
                min-width: 30px;
            }
            QDoubleSpinBox {
                background: #10141a;
                border: 1px solid #465262;
                border-radius: 5px;
                padding: 5px;
                min-height: 25px;
            }
        """


def main(args=None):
    # Qt owns the main loop, so handle SIGINT here instead of allowing the
    # default ROS handler to invalidate the context under an active Qt timer.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArmControlNode()
    app = QApplication(sys.argv)
    window = ArmControlWindow(node)
    window.show()
    signal.signal(signal.SIGINT, lambda _signum, _frame: app.quit())

    ros_timer = QTimer(window)

    def spin_ros():
        if not rclpy.ok():
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0)
        except KeyboardInterrupt:
            app.quit()
        except Exception:
            # The ROS signal handler can invalidate the context between the
            # check above and spin_once() during launch shutdown.
            if rclpy.ok():
                raise
            app.quit()

    ros_timer.timeout.connect(spin_ros)
    ros_timer.start(20)

    try:
        return app.exec_()
    finally:
        ros_timer.stop()
        if rclpy.ok():
            node.send_base(0.0)
            rclpy.spin_once(node, timeout_sec=0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
