#!/usr/bin/env python3
"""Qt control panel for full_arm_controller (shoulder, elbow, wrist PID).

Tuning tab: per-joint enable, target and Kp/Ki/Kd/G, live angle and duty plots.
IK tab: click a point on the side view; the tip follows a straight line there
while the panel streams intermediate IK solutions as joint targets.

Safety rules the panel enforces:
  * While a joint is disabled its target follows its measured angle, so
    enabling never jumps.
  * "Park & disable" drives every enabled joint to 90 deg before cutting
    power: a disabled joint far from vertical falls.
  * Gains are only sent while everything is disabled (the firmware ignores
    them otherwise).
"""

import sys
import time
from math import hypot

from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QSlider, QTabWidget, QVBoxLayout, QWidget)
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32

from manipulator_control import arm_kinematics as kin
from manipulator_control import qt_ros
from manipulator_control.estop import ESTOP_QOS, ESTOP_TOPIC
from manipulator_control.signal_plot import SignalPlot

JOINTS = ('shoulder', 'elbow', 'wrist')
COLORS = ('#4fc3f7', '#ffb74d', '#81c784')
# Must match full_arm_controller.ino defaults (joints[], gravity_gain[],
# friction_gain[]): Kp, Ki, Kd, G (gravity %), F (friction %).
DEFAULT_GAINS = ((3.0, 2.0, 0.1, 33.0, 10.0), (4.0, 1.0, 0.04, 28.0, 12.0),
                 (5.0, 2.0, 0.04, 25.0, 10.0))
# Firmware gainsCallback bounds.
GAIN_LIMITS = ((0.0, 5.0), (0.0, 2.0), (0.0, 1.0), (0.0, 80.0), (0.0, 40.0))
GAIN_LABELS = ('Kp', 'Ki', 'Kd', 'G %', 'F %')
FAULTS = {0: 'none', 1: 'stale/invalid target', 2: 'feedback invalid', 3: 'control timeout',
          4: 'e-stop', 5: 'runaway', 6: 'stall'}
TELEMETRY_TIMEOUT = 0.5
PARK_DEG = 90.0
PARK_TOLERANCE = 3.0
PARK_TIMEOUT = 20.0
TICK_S = 0.05
TIP_SPEED = 0.03          # m/s along the straight tip path
PHI_SPEED = 15.0          # deg/s tip-angle change
MAX_JOINT_RATE = 6.0      # deg/s per joint; below the firmware's 8 deg/s ramp


class ArmNode(Node):
    """ROS side: publishes targets/enable/gains/e-stop, stores telemetry."""

    def __init__(self):
        super().__init__('arm_control_gui')
        self.targets_pub = self.create_publisher(Float32MultiArray, '/arm_joint_targets_deg', 10)
        self.enable_pub = self.create_publisher(Int32, '/arm_pid_enable', 10)
        self.gains_pub = self.create_publisher(Float32MultiArray, '/arm_pid_gains', 10)
        self.estop_pub = self.create_publisher(Bool, ESTOP_TOPIC, ESTOP_QOS)
        self.telemetry = None
        self.telemetry_time = 0.0
        self.estop = False
        self.create_subscription(
            Float32MultiArray, '/arm_joint_telemetry', self._telemetry_cb, 10)
        self.create_subscription(Bool, ESTOP_TOPIC, self._estop_cb, ESTOP_QOS)

    def _telemetry_cb(self, msg):
        if len(msg.data) >= 16:
            self.telemetry = list(msg.data)
            self.telemetry_time = time.monotonic()

    def _estop_cb(self, msg):
        self.estop = msg.data

    def fresh(self):
        return self.telemetry is not None and time.monotonic() - self.telemetry_time < TELEMETRY_TIMEOUT

    def send_targets(self, targets):
        self.targets_pub.publish(Float32MultiArray(data=[float(t) for t in targets]))

    def send_enable(self, mask):
        self.enable_pub.publish(Int32(data=int(mask)))

    def send_gains(self, gains):
        self.gains_pub.publish(Float32MultiArray(data=[float(g) for row in gains for g in row]))

    def send_estop(self, engaged):
        self.estop_pub.publish(Bool(data=engaged))

    def shutdown(self):
        self.send_enable(0)


def clamp_target(i, value):
    lo, hi = kin.JOINT_LIMITS[i]
    return max(lo, min(hi, value))


class ArmView(QWidget):
    """Side view (reach x height) of the arm; click to pick an IK goal."""

    R_MIN, R_MAX, Z_MIN, Z_MAX = -0.12, 0.48, -0.02, 0.48

    def __init__(self, on_click):
        super().__init__()
        self.on_click = on_click
        self.measured = None
        self.target = None
        self.goal = None
        self.goal_ok = True
        self.setMinimumSize(420, 360)

    def _scale(self):
        sx = self.width() / (self.R_MAX - self.R_MIN)
        sz = self.height() / (self.Z_MAX - self.Z_MIN)
        return min(sx, sz)

    def to_px(self, r, z):
        s = self._scale()
        return QPointF((r - self.R_MIN) * s, self.height() - (z - self.Z_MIN) * s)

    def from_px(self, x, y):
        s = self._scale()
        return x / s + self.R_MIN, (self.height() - y) / s + self.Z_MIN

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.on_click(*self.from_px(event.x(), event.y()))

    def _draw_arm(self, p, q, color, dashed):
        pen = QPen(QColor(color), 5 if not dashed else 2)
        if dashed:
            pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        pts = [self.to_px(r, z) for r, z in kin.forward_points(q)]
        for a, b in zip(pts, pts[1:]):
            p.drawLine(a, b)
        p.setBrush(QColor(color))
        for pt in pts[:-1]:
            p.drawEllipse(pt, 4, 4)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#1d2227'))
        p.setPen(QPen(QColor('#323b44'), 1))
        for i in range(-1, 5):
            v = i * 0.1
            p.drawLine(self.to_px(v, self.Z_MIN), self.to_px(v, self.Z_MAX))
            p.drawLine(self.to_px(self.R_MIN, v), self.to_px(self.R_MAX, v))
        p.setPen(QPen(QColor('#9ba8b4'), 2))
        p.drawLine(self.to_px(self.R_MIN, 0.0), self.to_px(self.R_MAX, 0.0))
        p.setFont(QFont('Sans', 8))
        p.drawText(8, 16, 'reach (m) →   height (m) ↑   grid 0.1 m')
        if self.target is not None:
            self._draw_arm(p, self.target, '#ffb74d', dashed=True)
        if self.measured is not None:
            self._draw_arm(p, self.measured, '#4fc3f7', dashed=False)
        if self.goal is not None:
            p.setPen(QPen(QColor('#81c784' if self.goal_ok else '#e57373'), 2))
            p.setBrush(Qt.NoBrush)
            c = self.to_px(*self.goal)
            p.drawEllipse(c, 7, 7)
            p.drawLine(c + QPointF(-10, 0), c + QPointF(10, 0))
            p.drawLine(c + QPointF(0, -10), c + QPointF(0, 10))
        p.setPen(QColor('#9ba8b4'))
        p.drawText(8, self.height() - 8, 'solid: measured   dashed: target   +: goal')


class ArmControlWindow(QMainWindow):
    def __init__(self, node: ArmNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('Manipulator — tuning & IK')
        self.targets = [90.0, 90.0, 90.0]
        self.requested_mask = 0
        self.parking = None          # park deadline while parking, else None
        self.path_goal = None        # (r, z, phi, q) while streaming an IK path
        self.path_joint_mode = False  # straight line blocked: finishing in joint space
        self.auto_phi = True
        # Targets are unknown until the first telemetry sample: the arm may
        # already be enabled by another client, so never publish defaults.
        self.targets_seeded = False
        self.t0 = time.monotonic()

        central = QWidget()
        root = QVBoxLayout(central)
        root.addLayout(self._build_status_bar())
        tabs = QTabWidget()
        tabs.addTab(self._build_tuning_tab(), 'Tuning')
        tabs.addTab(self._build_ik_tab(), 'Inverse kinematics')
        root.addWidget(tabs)
        self.setCentralWidget(central)
        self.resize(1100, 820)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(TICK_S * 1000))

    # ── Layout ──────────────────────────────────────────────────────────────
    def _build_status_bar(self):
        bar = QHBoxLayout()
        self.status_label = QLabel('waiting for /arm_joint_telemetry…')
        self.status_label.setStyleSheet('font-weight: bold;')
        bar.addWidget(self.status_label, 1)
        self.enable_all_btn = QPushButton('Enable all')
        self.enable_all_btn.clicked.connect(lambda: self._request_mask(7))
        self.park_btn = QPushButton('Park at 90° && disable')
        self.park_btn.clicked.connect(self._start_park)
        self.disable_btn = QPushButton('Disable now')
        self.disable_btn.setToolTip('Cuts power immediately: a joint far from vertical will fall.')
        self.disable_btn.clicked.connect(lambda: self._request_mask(0))
        self.estop_btn = QPushButton('E-STOP')
        self.estop_btn.setStyleSheet('background:#c62828; color:white; font-weight:bold;')
        self.estop_btn.clicked.connect(self._toggle_estop)
        for w in (self.enable_all_btn, self.park_btn, self.disable_btn, self.estop_btn):
            bar.addWidget(w)
        return bar

    def _build_tuning_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        joints_row = QHBoxLayout()
        self.enable_boxes, self.sliders, self.target_spins = [], [], []
        self.measured_labels, self.gain_spins = [], []
        for i, name in enumerate(JOINTS):
            box = QGroupBox(name.capitalize())
            g = QGridLayout(box)
            enable = QCheckBox('enabled')
            enable.toggled.connect(self._enable_boxes_changed)
            g.addWidget(enable, 0, 0, 1, 2)
            measured = QLabel('—')
            g.addWidget(measured, 0, 2, 1, 2)
            lo, hi = kin.JOINT_LIMITS[i]
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(lo * 2), int(hi * 2))
            slider.valueChanged.connect(lambda v, i=i: self._set_target(i, v / 2.0))
            g.addWidget(slider, 1, 0, 1, 3)
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(1)
            spin.setSingleStep(0.5)
            spin.setSuffix('°')
            spin.editingFinished.connect(lambda i=i, s=spin: self._set_target(i, s.value()))
            g.addWidget(spin, 1, 3)
            spins = []
            for col, (label, (glo, ghi)) in enumerate(zip(GAIN_LABELS, GAIN_LIMITS)):
                g.addWidget(QLabel(label), 2, col)
                s = QDoubleSpinBox()
                s.setRange(glo, ghi)
                s.setDecimals(2)
                s.setSingleStep(0.1 if ghi <= 5 else 1.0)
                s.setValue(DEFAULT_GAINS[i][col])
                g.addWidget(s, 3, col)
                spins.append(s)
            self.enable_boxes.append(enable)
            self.sliders.append(slider)
            self.target_spins.append(spin)
            self.measured_labels.append(measured)
            self.gain_spins.append(spins)
            joints_row.addWidget(box)
        layout.addLayout(joints_row)

        gains_row = QHBoxLayout()
        self.send_gains_btn = QPushButton('Send gains to ESP32')
        self.send_gains_btn.clicked.connect(self._send_gains)
        reset = QPushButton('Reset to firmware defaults')
        reset.clicked.connect(self._reset_gains)
        self.gains_note = QLabel('Gains are accepted only while every joint is disabled.')
        gains_row.addWidget(self.send_gains_btn)
        gains_row.addWidget(reset)
        gains_row.addWidget(self.gains_note, 1)
        layout.addLayout(gains_row)

        self.angle_plot = SignalPlot(unit='°', window=20.0, title='Angle (solid) and target (dashed)')
        self.duty_plot = SignalPlot(unit='%', window=20.0, title='Motor duty')
        for name, color in zip(JOINTS, COLORS):
            self.angle_plot.add_series(name, color)
            self.angle_plot.add_series(f'{name} target', color, dashed=True)
            self.duty_plot.add_series(name, color)
        layout.addWidget(self.angle_plot, 3)
        layout.addWidget(self.duty_plot, 2)
        return tab

    def _build_ik_tab(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        self.view = ArmView(self._ik_goal_clicked)
        layout.addWidget(self.view, 3)
        side = QVBoxLayout()
        side.addWidget(QLabel('Click the view to set a goal for the gripper tip.'))
        self.auto_phi_box = QCheckBox('Choose tip angle automatically')
        self.auto_phi_box.setChecked(True)
        self.auto_phi_box.toggled.connect(self._auto_phi_changed)
        side.addWidget(self.auto_phi_box)
        side.addWidget(QLabel('Tip angle from vertical (0 up, 90 horizontal, 180 down)'))
        self.phi_slider = QSlider(Qt.Horizontal)
        self.phi_slider.setRange(0, 180)
        self.phi_slider.setValue(90)
        self.phi_slider.setEnabled(False)
        self.phi_slider.valueChanged.connect(self._phi_changed)
        side.addWidget(self.phi_slider)
        self.phi_label = QLabel('90°')
        side.addWidget(self.phi_label)
        self.goal_label = QLabel('goal: —')
        self.goal_label.setWordWrap(True)
        side.addWidget(self.goal_label)
        self.go_btn = QPushButton('Move along straight line')
        self.go_btn.clicked.connect(self._start_path)
        self.stop_path_btn = QPushButton('Stop motion (hold here)')
        self.stop_path_btn.clicked.connect(self._stop_path)
        side.addWidget(self.go_btn)
        side.addWidget(self.stop_path_btn)
        self.path_label = QLabel('Requires all three joints enabled.')
        self.path_label.setWordWrap(True)
        side.addWidget(self.path_label)
        side.addStretch(1)
        layout.addLayout(side, 1)
        self.ik_goal = None  # (r, z, phi, q)
        return tab

    # ── Telemetry helpers ───────────────────────────────────────────────────
    def _telemetry(self):
        return self.node.telemetry if self.node.fresh() else None

    def _mask(self):
        t = self._telemetry()
        return int(t[12]) if t else 0

    def _measured(self):
        t = self._telemetry()
        return t[:3] if t else None

    # ── Commands ────────────────────────────────────────────────────────────
    def _request_mask(self, mask):
        if mask == 0:
            self.parking = None
            self.path_goal = None
        self.requested_mask = mask
        self.node.send_targets(self.targets)
        self.node.send_enable(mask)
        for i, box in enumerate(self.enable_boxes):
            box.blockSignals(True)
            box.setChecked(bool(mask & (1 << i)))
            box.blockSignals(False)

    def _enable_boxes_changed(self):
        mask = sum(1 << i for i, b in enumerate(self.enable_boxes) if b.isChecked())
        self._request_mask(mask)

    def _set_target(self, i, value):
        self.path_goal = None
        self.targets[i] = clamp_target(i, value)
        self._sync_target_widgets(i)

    def _sync_target_widgets(self, i):
        for w, v in ((self.sliders[i], int(round(self.targets[i] * 2))),
                     (self.target_spins[i], self.targets[i])):
            w.blockSignals(True)
            w.setValue(v)
            w.blockSignals(False)

    def _start_park(self):
        if self._mask() == 0:
            return
        self.path_goal = None
        for i in range(3):
            self.targets[i] = PARK_DEG
            self._sync_target_widgets(i)
        self.parking = time.monotonic() + PARK_TIMEOUT

    def _toggle_estop(self):
        self.node.send_estop(not self.node.estop)
        self.node.estop = not self.node.estop

    def _send_gains(self):
        if self._mask() != 0:
            self.gains_note.setText('Not sent: disable every joint first.')
            return
        gains = [[s.value() for s in row] for row in self.gain_spins]
        self.node.send_gains(gains)
        self.gains_note.setText('Gains sent (firmware applies them only while disabled).')

    def _reset_gains(self):
        for row, defaults in zip(self.gain_spins, DEFAULT_GAINS):
            for s, v in zip(row, defaults):
                s.setValue(v)

    # ── IK ──────────────────────────────────────────────────────────────────
    def _auto_phi_changed(self, checked):
        self.auto_phi = checked
        self.phi_slider.setEnabled(not checked)
        if self.ik_goal:
            self._ik_goal_clicked(self.ik_goal[0], self.ik_goal[1])

    def _phi_changed(self, value):
        self.phi_label.setText(f'{value}°')
        if self.ik_goal and not self.auto_phi:
            self._ik_goal_clicked(self.ik_goal[0], self.ik_goal[1])

    def _ik_goal_clicked(self, r, z):
        prev = list(self.targets)
        if self.auto_phi:
            _, _, phi_now = kin.forward(prev)
            found = kin.solve_auto(r, z, prev, phi_hint=phi_now)
            q, phi = found if found else (None, None)
        else:
            phi = float(self.phi_slider.value())
            q = kin.solve(r, z, phi, prev)
        self.view.goal = (r, z)
        self.view.goal_ok = q is not None
        if q is None:
            self.ik_goal = (r, z, None, None)
            self.goal_label.setText(f'goal r={r:.3f} z={z:.3f}: unreachable within joint limits')
        else:
            self.ik_goal = (r, z, phi, q)
            if self.auto_phi:
                self.phi_slider.blockSignals(True)
                self.phi_slider.setValue(int(round(phi)))
                self.phi_slider.blockSignals(False)
                self.phi_label.setText(f'{phi:.0f}°')
            self.goal_label.setText(
                f'goal r={r:.3f} z={z:.3f} tip {phi:.0f}°\n'
                f'joints {q[0]:.1f} / {q[1]:.1f} / {q[2]:.1f}°')
        self.view.update()

    def _start_path(self):
        if not self.ik_goal or self.ik_goal[3] is None:
            self.path_label.setText('Pick a reachable goal first.')
            return
        if self._mask() != 7:
            self.path_label.setText('Enable all three joints first.')
            return
        self.parking = None
        self.path_goal = self.ik_goal
        self.path_joint_mode = False
        self.path_label.setText('Moving along a straight line…')

    def _stop_path(self):
        self.path_goal = None
        m = self._measured()
        if m:
            for i in range(3):
                self.targets[i] = clamp_target(i, m[i])
                self._sync_target_widgets(i)
        self.path_label.setText('Stopped; holding the measured pose.')

    def _move_joints_toward(self, q):
        """Step all joints toward q together, capped at MAX_JOINT_RATE."""
        deltas = [a - b for a, b in zip(q, self.targets)]
        biggest = max(abs(d) for d in deltas)
        scale = min(1.0, MAX_JOINT_RATE * TICK_S / biggest) if biggest > 0 else 1.0
        self.targets = [b + d * scale for b, d in zip(self.targets, deltas)]
        for i in range(3):
            self._sync_target_widgets(i)
        return biggest <= MAX_JOINT_RATE * TICK_S

    def _step_path(self):
        """Advance the streamed path by one tick.

        Follows a straight tip line while every point on it is solvable; if
        the line would cross a joint limit, finishes in joint space instead."""
        r_goal, z_goal, phi_goal, q_goal = self.path_goal
        if self.path_joint_mode:
            if self._move_joints_toward(q_goal):
                self.path_goal = None
                self.path_label.setText('Reached goal (joint-space move).')
            return
        r, z, phi = kin.forward(self.targets)
        dist = hypot(r_goal - r, z_goal - z)
        dphi = phi_goal - phi
        if dist < 1e-4 and abs(dphi) < 0.1:
            self._move_joints_toward(q_goal)
            self.path_goal = None
            self.path_label.setText('Reached goal.')
            return
        frac = 1.0
        if dist > 0:
            frac = min(frac, TIP_SPEED * TICK_S / dist)
        if abs(dphi) > 0:
            frac = min(frac, PHI_SPEED * TICK_S / abs(dphi))
        q = kin.solve(r + (r_goal - r) * frac, z + (z_goal - z) * frac,
                      phi + dphi * frac, self.targets)
        if q is None:
            self.path_joint_mode = True
            self.path_label.setText(
                f'Straight line would cross a joint limit near r={r:.3f} z={z:.3f}; '
                'finishing in joint space.')
            self._move_joints_toward(q_goal)
            return
        # Near vertical a small tip step can need a large joint step: the rate
        # cap slows the motion there instead of jumping.
        self._move_joints_toward(q)

    # ── 20 Hz loop ──────────────────────────────────────────────────────────
    def _tick(self):
        t = self._telemetry()
        now = time.monotonic() - self.t0
        mask = self._mask()
        if t is None:
            self.status_label.setText('No telemetry (ESP32 or micro-ROS agent offline)')
            self.status_label.setStyleSheet('font-weight:bold; color:#e57373;')
        else:
            fault, fjoint = int(t[13]), int(t[14])
            where = f' ({JOINTS[fjoint - 1]})' if fault and 1 <= fjoint <= 3 else ''
            state = 'E-STOP ENGAGED' if self.node.estop else (
                'parking' if self.parking else ('enabled ' + ''.join(
                    'SEW'[i] if mask & (1 << i) else '-' for i in range(3)) if mask else 'disabled'))
            self.status_label.setText(
                f'{state}   fault: {FAULTS.get(fault, fault)}{where}   '
                f'angles {t[0]:.1f} / {t[1]:.1f} / {t[2]:.1f}°')
            self.status_label.setStyleSheet(
                'font-weight:bold; color:%s;' % ('#e57373' if fault or self.node.estop else '#81c784'))
            for i in range(3):
                self.measured_labels[i].setText(f'{t[i]:.1f}°   duty {t[9 + i]:+.1f}%')
                if not self.targets_seeded or not mask & (1 << i):
                    # Seed from the arm, then keep disabled joints tracking it so
                    # a later enable never jumps.
                    if t[i] > 0:
                        self.targets[i] = clamp_target(i, t[i])
                        self._sync_target_widgets(i)
                self.angle_plot.push(JOINTS[i], now, t[i])
                self.angle_plot.push(f'{JOINTS[i]} target', now, self.targets[i])
                self.duty_plot.push(JOINTS[i], now, t[9 + i])
            # The firmware drops enable on any fault: keep the checkboxes honest.
            if mask != self.requested_mask and not self.parking:
                self.requested_mask = mask
                for i, box in enumerate(self.enable_boxes):
                    box.blockSignals(True)
                    box.setChecked(bool(mask & (1 << i)))
                    box.blockSignals(False)
                if mask == 0:
                    self.path_goal = None
        self.angle_plot.advance(now)
        self.duty_plot.advance(now)

        if self.path_goal is not None:
            if mask != 7:
                self.path_goal = None
                self.path_label.setText('Stopped: a joint was disabled.')
            else:
                self._step_path()

        if self.parking is not None:
            m = self._measured()
            done = m is not None and all(
                abs(m[i] - PARK_DEG) < PARK_TOLERANCE for i in range(3) if mask & (1 << i))
            if mask == 0 or done or time.monotonic() > self.parking:
                timed_out = not done and mask != 0
                self.parking = None
                self._request_mask(0)
                if timed_out:
                    self.status_label.setText('Park timed out; disabled anyway')

        self.send_gains_btn.setEnabled(mask == 0)
        self.estop_btn.setText('RELEASE E-STOP' if self.node.estop else 'E-STOP')
        self.view.measured = self._measured() if t and all(a > 0 for a in t[:3]) else None
        self.view.target = list(self.targets)
        self.view.update()
        if t is not None:
            self.targets_seeded = True
            self.node.send_targets(self.targets)


def main(args=None):
    return qt_ros.run(ArmNode, ArmControlWindow, on_exit=lambda node: node.shutdown(), args=args)


if __name__ == '__main__':
    sys.exit(main())
