#trends_panel.py
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore import Qt

TREND_METRICS = [
    ("Rain",        "mm"),
    ("High Temp", "°C"),
    ("Low Temp", "°C"),
    ("Wind",        "km/h"),
    ("Battery SOC", "%"),
    ("Battery V",   "V"),
    ("Tank 1 Out",  "L"),
    ("Human Days",  "days"),
    ("Tractor Use", "hrs"),
]

PERIODS = ["1d", "7d", "30d", "90d"]

STUB_DATA = {
    "Rain":        [2.4,  18.2,  44.0,  91.5],
    "High Temp":   [22.1, 21.4,  19.8,  18.2],
    "Low Temp":    [12.1, 11.4,  9.8,   8.2],
    "Wind":        [12.0, 14.5,  13.1,  11.8],
    "Battery SOC": [87.0, 84.2,  81.5,  79.0],
    "Battery V":   [52.1, 51.8,  51.2,  50.9],
    "Tank 1 Out":  [320,  2100,  8400,  31200],
    "Human Days":  [1,    5,     18,    52],
    "Tractor Use": [2.5,  11.0,  38.5,  104.0],
}

NAME_W  = 140
VALUE_W = 75


def _divider():
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #555;")
    return line


class TrendsPanel(QWidget):
    def __init__(self, parent, state):
        super().__init__(parent)
        self.state = state
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("""
            QWidget {
                background: rgba(20, 20, 20, 210);
                color: white;
                border-top: 1px solid #444;
            }
        """)

        main = QVBoxLayout(self)
        main.setContentsMargins(14, 8, 14, 8)
        main.setSpacing(4)

        # ── header row ───────────────────────────────────────────
        header_row = QHBoxLayout()
        header_row.setSpacing(0)

        spacer = QLabel("")
        spacer.setFixedWidth(NAME_W)
        header_row.addWidget(spacer)

        for period in PERIODS:
            lbl = QLabel(period)
            lbl.setStyleSheet("font-size: 11px; color: #888; letter-spacing: 1px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            lbl.setFixedWidth(VALUE_W)
            header_row.addWidget(lbl)

        main.addLayout(header_row)
        main.addWidget(_divider())

        # ── metric rows ──────────────────────────────────────────
        self._value_labels = {}

        for i, (metric, unit) in enumerate(TREND_METRICS):
            row = QHBoxLayout()
            row.setSpacing(0)

            name_lbl = QLabel(metric)
            name_lbl.setStyleSheet("font-size: 13px;")
            name_lbl.setFixedWidth(NAME_W)
            row.addWidget(name_lbl)

            self._value_labels[metric] = []
            values = STUB_DATA.get(metric, [None] * 4)

            for val in values:
                text = f"{val} {unit}" if val is not None else "--"
                v_lbl = QLabel(text)
                v_lbl.setStyleSheet("font-size: 13px; color: #ccc;")
                v_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                v_lbl.setFixedWidth(VALUE_W)
                row.addWidget(v_lbl)
                self._value_labels[metric].append(v_lbl)

            main.addLayout(row)

            if i < len(TREND_METRICS) - 1:
                sep = QFrame()
                sep.setFrameShape(QFrame.Shape.HLine)
                sep.setStyleSheet("color: #2a2a2a;")
                main.addWidget(sep)

        main.addStretch()

    def update_trends(self, data: dict):
        for metric, unit in TREND_METRICS:
            if metric not in data:
                continue
            values = data[metric]
            labels = self._value_labels.get(metric, [])
            for lbl, val in zip(labels, values):
                lbl.setText(f"{val} {unit}" if val is not None else "--")