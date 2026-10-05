#!/usr/bin/env python3
"""Qt analysis window: tracking error, gripper/base plots, pause and CSV export.

Read-only companion to arm_control_gui; it publishes nothing.

  Tracking error  command - measured per joint, degrees. The shoulder is
                  measured by the potentiometer when available, else the
                  /joint_states encoder; elbow and wrist only have encoders
                  (simulation).
  Gripper / base  commanded gripper position and base velocity, plus the
                  measured gripper closure when /joint_states carries it.
"""

import csv
import os
import sys
import time
from collections import deque
from datetime import datetime
from math import degrees

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from manipulator_control import qt_ros
from manipulator_control.arm_telemetry import ArmTelemetry
from manipulator_control.signal_plot import SignalPlot

SAMPLE_MS = 50
MAX_ROWS = 3600 * 1000 // SAMPLE_MS      # one hour of recording
WINDOWS = (10, 20, 60)

COLUMNS = (
    'time_s',
    'cmd_base', 'cmd_shoulder_deg', 'cmd_elbow_deg', 'cmd_wrist_deg',
    'cmd_gripper',
    'meas_shoulder_deg', 'meas_elbow_deg', 'meas_wrist_deg', 'meas_gripper',
    'err_shoulder_deg', 'err_elbow_deg', 'err_wrist_deg',
)

STYLE = """
    QMainWindow, QWidget { background: #151a20; color: #e8edf2; }
    QPushButton {
        background: #2a323b; border: 1px solid #465262; border-radius: 6px;
        padding: 7px 16px;
    }
    QPushButton:hover { background: #343e49; }
    QComboBox {
        background: #10141a; border: 1px solid #465262; border-radius: 5px;
        padding: 5px;
    }
    QLabel#status { color: #9ba8b4; }
"""


class ArmAnalysisWindow(QMainWindow):
    def __init__(self, node: ArmTelemetry):
        super().__init__()
        self.node = node
        self.paused = False
        self.rows = deque(maxlen=MAX_ROWS)
        self._t0 = time.monotonic()

        self.setWindowTitle('Manipulator Analysis')
        self.resize(900, 760)
        self.setStyleSheet(STYLE)

        central = QWidget()
        layout = QVBoxLayout(central)
        self.setCentralWidget(central)

        bar = QHBoxLayout()
        self.pause_button = QPushButton('Pause')
        self.pause_button.clicked.connect(self._toggle_pause)
        clear_button = QPushButton('Clear')
        clear_button.clicked.connect(self.clear)
        export_button = QPushButton('Export CSV…')
        export_button.clicked.connect(self._export_dialog)
        self.window_box = QComboBox()
        for seconds in WINDOWS:
            self.window_box.addItem(f'{seconds} s', seconds)
        self.window_box.setCurrentIndex(WINDOWS.index(20))
        self.window_box.currentIndexChanged.connect(self._window_changed)
        self.status = QLabel('')
        self.status.setObjectName('status')
        for widget in (self.pause_button, clear_button, export_button,
                       QLabel('Window'), self.window_box):
            bar.addWidget(widget)
        bar.addStretch(1)
        bar.addWidget(self.status)
        layout.addLayout(bar)

        self.error_plot = SignalPlot(
            unit='deg', window=20.0, title='Tracking error (command − measured)')
        for name, color in (('Shoulder', '#4cb4ff'), ('Elbow', '#ff5fa2'),
                            ('Wrist', '#ffc857')):
            self.error_plot.add_series(name, color)
        self.aux_plot = SignalPlot(
            unit='', window=20.0,
            title='Gripper (0 open..1 closed) and base velocity (−1..1)')
        self.aux_plot.add_series('Gripper cmd', '#2dd4bf', dashed=True)
        self.aux_plot.add_series('Gripper meas', '#2dd4bf')
        self.aux_plot.add_series('Base cmd', '#b794f6', dashed=True)
        self.plots = (self.error_plot, self.aux_plot)
        layout.addWidget(self.error_plot, 3)
        layout.addWidget(self.aux_plot, 2)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._sample)
        self.timer.start(SAMPLE_MS)

    # -- controls ---------------------------------------------------------

    def _toggle_pause(self):
        self.paused = not self.paused
        for plot in self.plots:
            plot.set_paused(self.paused)
            plot.update()
        self.pause_button.setText('Resume' if self.paused else 'Pause')

    def _window_changed(self, _index):
        seconds = self.window_box.currentData()
        for plot in self.plots:
            plot.set_window(seconds)

    def clear(self):
        self.rows.clear()
        for plot in self.plots:
            plot.clear()

    def _export_dialog(self):
        default = os.path.expanduser(
            f'~/arm_log_{datetime.now():%Y%m%d_%H%M%S}.csv')
        path, _ = QFileDialog.getSaveFileName(
            self, 'Export recording', default, 'CSV files (*.csv)')
        if path:
            count = self.export_csv(path)
            self.status.setText(f'Exported {count} rows to {path}')

    def export_csv(self, path):
        """Write every recorded sample (since start/clear) to `path`."""
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            for row in self.rows:
                writer.writerow(
                    '' if v is None else f'{v:.6f}' for v in row)
        return len(self.rows)

    # -- sampling ---------------------------------------------------------

    def _sample(self):
        if self.paused:
            return
        now = time.monotonic()
        node = self.node
        cmd = node.last_command if node.is_fresh(node.last_command_time, now) else None
        meas = node.measured_joints()
        grip_meas = (node.last_gripper
                     if node.is_fresh(node.last_joints_time, now) else None)

        cmd_deg = [None] * 3
        err = [None] * 3
        if cmd is not None:
            cmd_deg = [degrees(v) for v in cmd[1:4]]
        meas_deg = [None if v is None else degrees(v) for v in meas]
        for i, name in enumerate(('Shoulder', 'Elbow', 'Wrist')):
            if cmd_deg[i] is not None and meas_deg[i] is not None:
                err[i] = cmd_deg[i] - meas_deg[i]
                self.error_plot.push(name, now, err[i])
        if cmd is not None:
            self.aux_plot.push('Gripper cmd', now, cmd[4])
            self.aux_plot.push('Base cmd', now, cmd[0])
        if grip_meas is not None:
            self.aux_plot.push('Gripper meas', now, grip_meas)

        for plot in self.plots:
            plot.advance(now)

        if cmd is not None or any(v is not None for v in meas_deg):
            self.rows.append((
                now - self._t0,
                *((cmd[0], *cmd_deg, cmd[4]) if cmd else (None,) * 5),
                *meas_deg, grip_meas, *err,
            ))
        self.status.setText(
            f'{len(self.rows)} samples recorded' + ('  [paused]' if self.paused else ''))


def main(args=None):
    return qt_ros.run(
        lambda: ArmTelemetry('arm_analysis_gui'), ArmAnalysisWindow, args=args)


if __name__ == '__main__':
    sys.exit(main())
