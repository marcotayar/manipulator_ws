"""Dependency-free rolling time-series plot for the Qt panel (QPainter only)."""

from collections import deque
from math import ceil, floor, log10

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget

BACKGROUND = QColor('#1d2227')
GRID = QColor('#323b44')
AXIS_TEXT = QColor('#9ba8b4')
LEGEND_BG = QColor(29, 34, 39, 220)


def _nice_step(span, target_ticks=5):
    raw = span / target_ticks
    mag = 10 ** floor(log10(raw))
    for mult in (1, 2, 5, 10):
        if raw <= mult * mag:
            return mult * mag
    return 10 * mag


class SignalPlot(QWidget):
    """Plots named series against the last `window` seconds.

    Series are declared with add_series(); samples are pushed with
    push(name, t, value). Clicking a legend entry shows/hides that series.
    A series with no sample newer than `stale_after` seconds is not drawn
    past its last sample, so a dead sensor shows as a flat gap, not a line.
    """

    def __init__(self, unit='', window=20.0, retain=60.0, title='', parent=None):
        super().__init__(parent)
        self.unit = unit
        self.window = window
        self.retain = max(retain, window)   # seconds of history kept
        self.title = title
        self.paused = False
        self._limits = []  # (value, QColor, series name)
        self.stale_after = 1.0
        self._series = {}  # name -> dict(color, data, visible)
        self._legend_hit = []  # [(QRectF, name)]
        self._now = 0.0
        self.setMinimumHeight(230)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def add_series(self, name, color, dashed=False):
        self._series[name] = {
            'color': QColor(color), 'dashed': dashed,
            'data': deque(), 'visible': True,
        }

    def add_limit(self, series, value, color=None):
        """Dotted horizontal line at `value`, shown while `series` is visible."""
        color = QColor(color) if color else QColor(self._series[series]['color'])
        self._limits.append((value, color, series))

    def set_window(self, seconds):
        self.window = seconds
        self.retain = max(self.retain, seconds)
        self.update()

    def set_paused(self, paused):
        self.paused = paused

    def clear(self):
        for s in self._series.values():
            s['data'].clear()
        self.update()

    def push(self, name, t, value):
        if self.paused:
            return
        data = self._series[name]['data']
        data.append((t, value))
        self._now = max(self._now, t)
        cutoff = self._now - self.retain
        while data and data[0][0] < cutoff:
            data.popleft()

    def advance(self, now):
        """Move the time axis forward even when no new samples arrived."""
        if not self.paused:
            self._now = max(self._now, now)
        self.update()

    def _y_range(self):
        t_min = self._now - self.window
        values = [v for s in self._series.values() if s['visible']
                  for t, v in s['data'] if t >= t_min]
        if values:
            values += [v for v, _, name in self._limits
                       if self._series[name]['visible']]
        if not values:
            return -1.0, 1.0
        lo, hi = min(values), max(values)
        if hi - lo < 1e-6:
            lo, hi = lo - 1.0, hi + 1.0
        pad = (hi - lo) * 0.1
        lo, hi = lo - pad, hi + pad
        step = _nice_step(hi - lo)
        return floor(lo / step) * step, ceil(hi / step) * step

    def mousePressEvent(self, event):
        for rect, name in self._legend_hit:
            if rect.contains(event.pos()):
                s = self._series[name]
                s['visible'] = not s['visible']
                self.update()
                return

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BACKGROUND)
        font = QFont(self.font())
        font.setPointSize(9)
        p.setFont(font)

        left, right, top, bottom = 52, 14, 24, 28
        plot = QRectF(left, top, self.width() - left - right,
                      self.height() - top - bottom)
        if plot.width() < 10 or plot.height() < 10:
            return

        y_lo, y_hi = self._y_range()
        t_hi = self._now
        t_lo = t_hi - self.window

        def to_px(t, v):
            x = plot.left() + (t - t_lo) / self.window * plot.width()
            y = plot.bottom() - (v - y_lo) / (y_hi - y_lo) * plot.height()
            return QPointF(x, y)

        # Grid + axis labels
        y_step = _nice_step(y_hi - y_lo)
        p.setPen(QPen(GRID, 1))
        v = y_lo
        while v <= y_hi + 1e-9:
            y = to_px(t_lo, v).y()
            p.setPen(QPen(GRID, 1))
            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setPen(AXIS_TEXT)
            p.drawText(QRectF(0, y - 9, left - 6, 18),
                       Qt.AlignRight | Qt.AlignVCenter, f'{v:g}')
            v += y_step
        x_step = _nice_step(self.window)
        k = 0.0
        while k <= self.window + 1e-9:
            x = plot.right() - k / self.window * plot.width()
            p.setPen(QPen(GRID, 1))
            p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))
            p.setPen(AXIS_TEXT)
            label = 'now' if k == 0 else f'-{k:g}s'
            p.drawText(QRectF(x - 24, plot.bottom() + 4, 48, 18),
                       Qt.AlignCenter, label)
            k += x_step
        p.setPen(AXIS_TEXT)
        if self.unit:
            p.drawText(QRectF(4, 2, left, 16), Qt.AlignLeft, self.unit)
        if self.title:
            p.drawText(QRectF(left, 2, plot.width(), 16), Qt.AlignCenter,
                       self.title + ('  [paused]' if self.paused else ''))

        # Limits
        p.save()
        p.setClipRect(plot)
        for value, color, name in self._limits:
            if not self._series[name]['visible']:
                continue
            pen = QPen(color, 1, Qt.DotLine)
            p.setPen(pen)
            y = to_px(t_lo, value).y()
            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
        p.restore()

        # Series
        p.save()
        p.setClipRect(plot)
        for s in self._series.values():
            if not s['visible'] or not s['data']:
                continue
            pen = QPen(s['color'], 2)
            if s['dashed']:
                pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            segment = QPolygonF()
            prev_t = None
            for t, val in s['data']:
                if prev_t is not None and t - prev_t > self.stale_after:
                    p.drawPolyline(segment)
                    segment = QPolygonF()
                segment.append(to_px(t, val))
                prev_t = t
            p.drawPolyline(segment)
        p.restore()

        # Legend (clickable)
        self._legend_hit = []
        row_h = 18
        names = list(self._series)
        width = 20 + max(
            (p.fontMetrics().horizontalAdvance(n) for n in names), default=0) + 12
        legend = QRectF(plot.left() + 8, plot.top() + 6, width,
                        row_h * len(names) + 8)
        p.setPen(Qt.NoPen)
        p.setBrush(LEGEND_BG)
        p.drawRoundedRect(legend, 5, 5)
        for i, name in enumerate(names):
            s = self._series[name]
            y = legend.top() + 4 + i * row_h
            color = s['color'] if s['visible'] else QColor('#5b6671')
            p.setBrush(color)
            p.drawRoundedRect(QRectF(legend.left() + 8, y + 4, 10, 10), 2, 2)
            p.setPen(QColor('#e8edf2') if s['visible'] else QColor('#5b6671'))
            p.drawText(QRectF(legend.left() + 24, y, width - 24, row_h),
                       Qt.AlignLeft | Qt.AlignVCenter, name)
            p.setPen(Qt.NoPen)
            self._legend_hit.append(
                (QRectF(legend.left(), y, width, row_h), name))
