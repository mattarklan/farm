from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QFrame
from PyQt6.QtCore import Qt


def _divider():
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #555;")
    return line


class StatusPanel(QWidget):
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
        main.setSpacing(6)

        # ── ASSET ────────────────────────────────────────────────
        asset_header = QLabel("SELECTED ASSET")
        asset_header.setStyleSheet("font-size: 12px; color: #888; letter-spacing: 1px;")

        self.asset_name = QLabel("None")
        self.asset_name.setStyleSheet("font-size: 15px; font-weight: bold;")

        self.asset_body = QLabel("\n\n\n\n\n\n")
        self.asset_body.setStyleSheet("font-size: 14px; color: #ccc;")
        self.asset_body.setWordWrap(False)
        self.asset_body.setFixedHeight(120)
        self.asset_body.setAlignment(Qt.AlignmentFlag.AlignTop)

        main.addWidget(asset_header)
        main.addWidget(self.asset_name)
        main.addWidget(self.asset_body)
        main.addWidget(_divider())

        # ── SYSTEM ───────────────────────────────────────────────
        sys_header = QLabel("SYSTEM")
        sys_header.setStyleSheet("font-size: 12px; color: #888; letter-spacing: 1px;")

        self.battery = QLabel("Battery SOC: --")
        self.voltage  = QLabel("Battery V: --")
        self.tank     = QLabel("Tank 1: --")
        self.temp     = QLabel("Temp: --")

        main.addWidget(sys_header)
        for lbl in (self.battery, self.voltage, self.tank, self.temp):
            lbl.setStyleSheet("font-size: 15px;")
            main.addWidget(lbl)

        main.addWidget(_divider())

        # ── GPS ──────────────────────────────────────────────────
        self.gps = QLabel("--")
        self.gps.setStyleSheet("font-size: 15px; font-weight: bold;")
        main.addWidget(self.gps)

        main.addStretch()

    def update_gps(self, gps):
        self.gps.setText(f"{gps[0]:.6f},  {gps[1]:.6f}" if gps else "--")

    def update_system(self, d):
        self.battery.setText(f"Battery SOC: {d.get('battery_soc', '--')}")
        self.voltage.setText(f"Battery V:   {d.get('battery_voltage', '--')}")
        self.tank.setText(f"Tank 1:      {d.get('tank_1_level', '--')}")
        self.temp.setText(f"Temp:        {d.get('temperature', '--')}")

    def update_asset(self, asset):
        if not asset:
            self.asset_name.setText("None")
            self.asset_body.setText("\n\n\n\n\n\n")
            return
        self.asset_name.setText(asset.name)
        if not asset.sensors:
            self.asset_body.setText("No sensor data\n\n\n\n\n\n")
            return
        lines = list(asset.sensors.items())[:7]
        text = "\n".join(f"{k}: {v}" for k, v in lines)
        shortage = 7 - len(lines)
        if shortage:
            text += "\n" * shortage
        self.asset_body.setText(text)