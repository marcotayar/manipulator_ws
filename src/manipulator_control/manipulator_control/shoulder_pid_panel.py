"""Explicit, telemetry-acknowledged shoulder PID tuning for the Qt panel."""

import time
from math import isclose, isfinite

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QVBoxLayout, QWidget,
)

from manipulator_control.signal_plot import SignalPlot


TARGET_MIN = 5.0
TARGET_MAX = 70.0
GAIN_DEFAULTS = (0.6, 0.1, 0.04)
GAIN_MAXIMA = (5.0, 2.0, 1.0)
TELEMETRY_TIMEOUT = 1.0
FAULTS = {
    0: 'none',
    1: 'invalid or stale target',
    2: 'invalid or outside-safe feedback',
    3: 'control timeout',
    4: 'emergency stop',
}


def valid_telemetry(values):
    """Reject malformed frames rather than treating them as fresh feedback."""
    return (
        len(values) == 14
        and all(isfinite(value) for value in values)
        and values[7] in (0.0, 1.0)
        and values[8] in (0.0, 1.0)
        and all(0.0 <= value <= maximum
                for value, maximum in zip(values[9:12], GAIN_MAXIMA))
        and 0.0 < values[12] <= 15.0
        and values[13] in FAULTS
    )


class ShoulderPidPanel(QScrollArea):
    """ROS callbacks and timers run on qt_ros's single Qt event loop."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self._telemetry = None
        self._received_at = None
        self._invalid_frame = False
        self._stale_disabled = False
        self._target_edited = False
        self._intent = False
        self._phase = 'idle'
        self._phase_started = None
        self._target_sent_at = None
        self._pending_gains = None
        self._gains_sent_at = None
        self._manual_target_pending = None
        self._closed = False
        self.setWidgetResizable(True)
        body = QWidget()
        layout = QVBoxLayout(body)
        layout.setContentsMargins(12, 12, 12, 12)
        self.setWidget(body)

        note = QLabel(
            'Real shoulder firmware telemetry required. Starts disabled; '
            'Enable first confirms the target, then requests PID. '
            'Stop coasts: support the arm against gravity.')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.status = QLabel('STALE — waiting for /shoulder_pid_telemetry')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        controls = QGridLayout()
        self.target = self._spin(TARGET_MIN, TARGET_MAX, 1, 0.5, TARGET_MIN)
        self.target.setSuffix(' deg')
        self.target.setToolTip(
            'The first Enable uses the latest measured angle unless you edit '
            'this target. While enabled, edits update the 10 Hz target stream.')
        self.target.valueChanged.connect(self._target_changed)
        self.target.lineEdit().textEdited.connect(self._target_changed)
        self.target.editingFinished.connect(self._send_target)
        controls.addWidget(QLabel('Target (deg)'), 0, 0)
        controls.addWidget(self.target, 0, 1)
        self.send_button = QPushButton('Send target')
        self.send_button.clicked.connect(self._send_target)
        controls.addWidget(self.send_button, 0, 2)
        self.enable_button = QPushButton('Enable PID')
        self.enable_button.clicked.connect(self._enable)
        self.stop_button = QPushButton('Stop PID')
        self.stop_button.clicked.connect(self.stop)
        controls.addWidget(self.enable_button, 0, 3)
        controls.addWidget(self.stop_button, 0, 4)
        self.gains = []
        for column, (name, maximum, initial, unit) in enumerate(zip(
                ('Kp', 'Ki', 'Kd'), GAIN_MAXIMA, GAIN_DEFAULTS,
                ('% / deg', '% / (deg s)', '% s / deg'))):
            spin = self._spin(0.0, maximum, 3, 0.01, initial)
            spin.setToolTip(f'{name}: {unit}; accepted only while disabled')
            self.gains.append(spin)
            controls.addWidget(QLabel(name), 1, column)
            controls.addWidget(spin, 2, column)
        self.apply_button = QPushButton('Apply gains')
        self.apply_button.clicked.connect(self._apply_gains)
        controls.addWidget(self.apply_button, 2, 3, 1, 2)
        layout.addLayout(controls)
        self.confirmed_label = QLabel('Firmware gains: not received')
        self.pending_label = QLabel('No gain request pending')
        self.latest_label = QLabel('Error: — deg | Applied duty: — %')
        self.action_label = QLabel('No target stream. PID is not enabled by this panel.')
        for label in (self.confirmed_label, self.pending_label,
                      self.latest_label, self.action_label):
            label.setWordWrap(True)
            layout.addWidget(label)

        self.angle_plot = SignalPlot(unit='deg', title='Target and measured angle')
        self.angle_plot.add_series('Target', '#4cb4ff', True)
        self.angle_plot.add_series('Measured', '#2dd4bf')
        self.error_plot = SignalPlot(unit='deg', title='Control error')
        self.error_plot.add_series('Error', '#ffc857')
        self.duty_plot = SignalPlot(unit='%', title='PID contributions and applied duty')
        for name, color in (('P', '#4cb4ff'), ('I', '#ff5fa2'),
                            ('D', '#ffc857'), ('Output', '#2dd4bf')):
            self.duty_plot.add_series(name, color)
        # Bounds are telemetry series too, so a stale board never gains samples.
        self.duty_plot.add_series('+Max duty', '#9ba8b4', True)
        self.duty_plot.add_series('-Max duty', '#9ba8b4', True)
        self.plots = (self.angle_plot, self.error_plot, self.duty_plot)
        for plot in self.plots:
            plot.setMinimumHeight(200)
            layout.addWidget(plot)

        self.node.pid_telemetry_callback = self._receive_telemetry
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)
        self.stream_timer = QTimer(self)
        self.stream_timer.setInterval(100)
        self.stream_timer.timeout.connect(self._publish_target)
        self._refresh_ui(time.monotonic())

    @staticmethod
    def _spin(minimum, maximum, decimals, step, initial):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(initial)
        spin.setKeyboardTracking(False)
        return spin

    def _fresh(self, now):
        return (not self._invalid_frame and self._received_at is not None
                and now - self._received_at <= TELEMETRY_TIMEOUT)

    def _safe(self, now):
        data = self._telemetry
        return (self._fresh(now) and not self.node.estop_active and not data[8]
                and TARGET_MIN <= data[1] <= TARGET_MAX)

    def _target_changed(self, _value):
        self._target_edited = True
        # A pre-enable target edit must receive its own acknowledgement.
        if self._intent and self._phase == 'target_ack':
            self._target_sent_at = None
            self._phase_started = time.monotonic()

    def _send_target(self, _checked=False):
        now = time.monotonic()
        if (self._closed or not self._fresh(now) or self.node.estop_active
                or self._telemetry[8]):
            self.action_label.setText('Target not sent: telemetry stale or emergency stop active.')
            return
        value = self.target.value()
        self._target_edited = True
        self.node.send_pid_target(value)
        self._manual_target_pending = (value, now)
        self.action_label.setText(
            f'Target {value:.1f} deg sent; waiting for firmware acknowledgement. '
            'Sending a target alone does not enable PID.')

    def _enable(self):
        now = time.monotonic()
        if (self._closed or self._intent or not self._safe(now)
                or self._telemetry[7] or self._pending_gains is not None):
            return
        if not self._target_edited:
            self.target.blockSignals(True)
            self.target.setValue(self._telemetry[1])
            self.target.blockSignals(False)
        self._intent = True
        self._phase = 'target_ack'
        self._phase_started = now
        self._target_sent_at = None
        self.action_label.setText('Waiting for firmware target acknowledgement; PID not requested yet.')
        self._publish_target()
        self.stream_timer.start()
        self._refresh_ui(now)

    def _publish_target(self):
        if not self._intent or self._closed:
            return
        now = time.monotonic()
        if not self._safe(now):
            self.stop(reason='Target stream cancelled: stale, stopped, or unsafe telemetry.')
            return
        self.node.send_pid_target(self.target.value())
        if self._target_sent_at is None:
            self._target_sent_at = now

    def stop(self, _checked=False, *, reason='Stop requested; target stream stopped.'):
        self.stream_timer.stop()
        self._intent = False
        self._phase = 'idle'
        self._phase_started = None
        self._target_sent_at = None
        self._manual_target_pending = None
        self.node.send_pid_enable(False)
        self.action_label.setText(reason)
        self._refresh_ui(time.monotonic())

    def emergency_stop(self):
        self.stop(reason='Emergency stop: target stream and pending enable cancelled.')

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self.stop(reason='Panel closed; PID disable requested.')
        self.timer.stop()
        self.node.pid_telemetry_callback = None

    def _apply_gains(self):
        now = time.monotonic()
        if (self._closed or not self._safe(now) or self._telemetry[7]
                or self._intent or self._pending_gains is not None):
            return
        self._pending_gains = tuple(spin.value() for spin in self.gains)
        self._gains_sent_at = now
        self.node.send_pid_gains(self._pending_gains)
        self.pending_label.setText(
            'Pending (not confirmed): Kp={:.3f}, Ki={:.3f}, Kd={:.3f}'.format(
                *self._pending_gains))
        self._refresh_ui(now)

    def _receive_telemetry(self, data, received_at):
        if self._closed:
            return
        if data is None:
            if not self._invalid_frame:
                self._invalid_frame = True
                self._stale_disabled = True
                self.stop(reason='Malformed telemetry; PID disable requested.')
            self._refresh_ui(received_at)
            return
        self._invalid_frame = False
        self._stale_disabled = False
        self._telemetry = data
        self._received_at = received_at
        for name, value in zip(('Target', 'Measured'), data[:2]):
            self.angle_plot.push(name, received_at, value)
        self.error_plot.push('Error', received_at, data[2])
        for name, value in zip(('P', 'I', 'D', 'Output'), data[3:7]):
            self.duty_plot.push(name, received_at, value)
        self.duty_plot.push('+Max duty', received_at, data[12])
        self.duty_plot.push('-Max duty', received_at, -data[12])
        self.confirmed_label.setText(
            'Firmware confirmed: Kp={:.3f}, Ki={:.3f}, Kd={:.3f} | max duty ±{:.2f}%'.format(
                *data[9:13]))
        self.latest_label.setText(
            f'Error: {data[2]:+.2f} deg | Applied duty: {data[6]:+.2f}%')
        if (self._pending_gains is not None and received_at > self._gains_sent_at
                and all(isclose(actual, requested, rel_tol=1e-5, abs_tol=1e-5)
                        for actual, requested in zip(data[9:12], self._pending_gains))):
            self._pending_gains = None
            self.pending_label.setText('Gain request acknowledged by firmware telemetry.')
        if self._manual_target_pending is not None:
            value, sent_at = self._manual_target_pending
            if received_at > sent_at and abs(data[0] - value) <= 0.02:
                self._manual_target_pending = None
                if not self._intent:
                    self.action_label.setText(
                        f'Firmware acknowledged target {value:.1f} deg; '
                        'click Enable PID separately to start control.')
        if self._intent:
            if not self._safe(received_at):
                self.stop(reason='Firmware fault or emergency stop; target stream cancelled.')
            elif self._phase == 'target_ack':
                if data[7]:
                    self.stop(reason='Unexpected enabled state before target acknowledgement.')
                elif (self._target_sent_at is not None
                      and received_at > self._target_sent_at
                      and abs(data[0] - self.target.value()) <= 0.02):
                    self.node.send_pid_enable(True)
                    self._phase = 'enable_ack'
                    self._phase_started = received_at
            elif self._phase == 'enable_ack' and data[7] and data[13] == 0:
                self._phase = 'running'
                self.action_label.setText('Firmware enabled; target streaming at 10 Hz.')
            elif self._phase == 'running' and (not data[7] or data[13] != 0):
                self.stop(reason='Firmware disabled or faulted; explicit Enable is required again.')
        self._refresh_ui(received_at)

    def _tick(self):
        now = time.monotonic()
        if (self._received_at is not None and not self._fresh(now)
                and not self._stale_disabled):
            self._stale_disabled = True
            self.stop(reason='Telemetry stale; PID disable requested. No automatic restart.')
        if self._intent:
            if not self._safe(now):
                self.stop(reason='Telemetry stale or unsafe; PID disable requested. No automatic restart.')
            elif (self._phase in ('target_ack', 'enable_ack')
                  and now - self._phase_started > TELEMETRY_TIMEOUT):
                self.stop(reason='Acknowledgement timed out; PID disable requested.')
        if (self._pending_gains is not None
                and now - self._gains_sent_at > TELEMETRY_TIMEOUT):
            self._pending_gains = None
            self.pending_label.setText('Gain request unconfirmed (timeout); see firmware confirmed values.')
        if (self._manual_target_pending is not None
                and now - self._manual_target_pending[1] > TELEMETRY_TIMEOUT):
            self._manual_target_pending = None
            if not self._intent:
                self.action_label.setText('Target acknowledgement timed out; target not confirmed.')
        for plot in self.plots:
            plot.advance(now)
        self._refresh_ui(now)

    def _refresh_ui(self, now):
        data = self._telemetry
        fresh = self._fresh(now)
        if self.node.estop_active or (fresh and data[8]):
            state = 'ESTOP'
        elif self._invalid_frame:
            state = 'INVALID TELEMETRY'
        elif not fresh:
            state = 'STALE — no valid telemetry within 1 s'
        elif data[13]:
            state = f'FAULT {int(data[13])}: {FAULTS[data[13]]}'
        else:
            state = 'ENABLED' if data[7] else 'DISABLED'
        if fresh:
            state += f' | firmware {"enabled" if data[7] else "disabled"}'
        self.status.setText(state)
        ready = self._safe(now) and not self._closed
        disabled = ready and not data[7] and not self._intent
        self.enable_button.setEnabled(disabled and self._pending_gains is None)
        self.send_button.setEnabled(
            fresh and not self._closed and not self.node.estop_active and not data[8])
        self.apply_button.setEnabled(disabled and self._pending_gains is None)
        for spin in self.gains:
            spin.setEnabled(disabled and self._pending_gains is None)
        self.target.setEnabled(not self._closed and self._phase != 'enable_ack')
