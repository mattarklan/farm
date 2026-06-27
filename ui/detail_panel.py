from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QHBoxLayout, QPushButton, QScrollArea
)
from PyQt6.QtCore import Qt


class DetailPanel(QWidget):
    MARGIN = 16

    def __init__(self, view):
        super().__init__(view.viewport())
        self._view = view

        self._build_ui()
        self.hide()

        view.viewport().installEventFilter(self)

    # ------------------------------------------------------------
    # PUBLIC: asset display
    # ------------------------------------------------------------

    def show_asset(self, asset):
        self._title.setText(asset.name)

        self._clear_rows()

        sensors = getattr(asset, "sensors", []) or []

        # fallback if YAML only gives names
        if sensors and isinstance(sensors[0], str):
            sensors = [
                {"name": s, "value": "—", "status": "unknown"}
                for s in sensors
            ]

        for s in sensors:
            self._add_sensor_row(s)

        self._reposition()
        self.show()
        self.raise_()

    # ------------------------------------------------------------
    # SENSOR ROWS
    # ------------------------------------------------------------

    def _add_sensor_row(self, sensor):
        name = sensor.get("name", "unknown")
        value = sensor.get("value", "—")
        status = sensor.get("status", "ok")

        row = QHBoxLayout()

        label = QLabel(name)
        label.setStyleSheet(self._status_style(status))

        val = QLabel(str(value))
        val.setAlignment(Qt.AlignmentFlag.AlignRight)

        row.addWidget(label)
        row.addStretch()
        row.addWidget(val)

        container = QWidget()
        container.setLayout(row)

        self._rows_layout.addWidget(container)

    def _status_style(self, status):
        if status == "warning":
            return "color: #FFD400; font-size:12px;"
        if status == "fault":
            return "color: #FF4444; font-size:12px;"
        return "color: #222; font-size:12px;"

    # ------------------------------------------------------------
    # CLEAR
    # ------------------------------------------------------------

    def _clear_rows(self):
        while self._rows_layout.count():
            item = self._rows_layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

    # ------------------------------------------------------------
    # UI BUILD
    # ------------------------------------------------------------

    def _build_ui(self):
        self.setObjectName("DetailPanel")
        self.setMinimumWidth(280)

        self.setStyleSheet("""
        #DetailPanel {
            background: rgba(240,240,240,245);
            border: 1px solid #888;
        }
        QLabel { background: transparent; }
        """)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)

        # HEADER
        header = QHBoxLayout()

        self._title = QLabel("Asset")
        self._title.setStyleSheet("font-size:14px; font-weight:bold;")

        close = QPushButton("✕")
        close.setFixedSize(20, 20)
        close.clicked.connect(self.hide)

        header.addWidget(self._title)
        header.addStretch()
        header.addWidget(close)

        outer.addLayout(header)

        # GLOBAL STATUS BAR (always visible)
        self._status_bar = QLabel("Battery: 88%   |   Weather: 22°C")
        self._status_bar.setStyleSheet(
            "color:#444; font-size:11px; padding:4px;"
        )
        outer.addWidget(self._status_bar)

        # divider
        div = QLabel()
        div.setFixedHeight(1)
        div.setStyleSheet("background:#aaa;")
        outer.addWidget(div)

        # scroll area
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)

        self._content = QWidget()
        self._rows_layout = QVBoxLayout(self._content)

        self._scroll.setWidget(self._content)
        outer.addWidget(self._scroll)

    # ------------------------------------------------------------
    # POSITIONING
    # ------------------------------------------------------------

    def _reposition(self):
        vp = self._view.viewport()

        x = vp.width() - self.width() - self.MARGIN
        y = vp.height() - self.height() - self.MARGIN

        self.move(max(0, x), max(0, y))

    def eventFilter(self, obj, event):
        if obj is self._view.viewport():
            if event.type() == event.Type.Resize:
                if self.isVisible():
                    self._reposition()
        return False