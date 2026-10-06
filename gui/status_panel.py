from PyQt6.QtWidgets import (
    QWidget,
    QLabel,
    QVBoxLayout,
    QFrame,
    QScrollArea,
)
from PyQt6.QtCore import (
    Qt,
    QThread,
    QObject,
    pyqtSignal,
)

from .mini_trend import MiniTrend
from .trend_client import TrendClient


def _divider():
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #555;")
    return line


class TrendWorker(QObject):

    finished = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(
        self,
        trend_client,
        asset_id,
        metric,
    ):
        super().__init__()

        self.trend_client = trend_client
        self.asset_id = asset_id
        self.metric = metric


    def run(self):

        try:

            points = (
                self.trend_client
                .get_hourly_metric(
                    self.asset_id,
                    self.metric,
                )
            )

            self.finished.emit(
                points
            )

        except Exception as exc:

            self.failed.emit(
                str(exc)
            )


class StatusPanel(QWidget):

    def __init__(
        self,
        parent,
        state,
        api_client,
    ):

        super().__init__(
            parent
        )

        self.state = state

        self.trend_client = TrendClient(
            api_client
        )

        self._trend_thread = None
        self._trend_worker = None

        self.setAttribute(
            Qt.WidgetAttribute.WA_StyledBackground,
            True
        )

        self.setStyleSheet(
            """
            QWidget {
                background: rgba(20, 20, 20, 210);
                color: white;
                border-top: 1px solid #444;
            }
            """
        )


        main = QVBoxLayout(
            self
        )

        main.setContentsMargins(
            14,
            8,
            14,
            8,
        )

        main.setSpacing(
            6
        )


        # --------------------------------------------------------
        # Selected asset
        # --------------------------------------------------------

        asset_header = QLabel(
            "SELECTED ASSET"
        )

        asset_header.setStyleSheet(
            "font-size: 12px; color: #888; letter-spacing: 1px;"
        )


        self.asset_name = QLabel(
            "None"
        )

        self.asset_name.setStyleSheet(
            "font-size: 15px; font-weight: bold;"
        )


        main.addWidget(
            asset_header
        )

        main.addWidget(
            self.asset_name
        )


        # --------------------------------------------------------
        # Current sensor values
        # --------------------------------------------------------

        self.asset_body = QLabel(
            "No asset selected"
        )

        self.asset_body.setStyleSheet(
            "font-size: 14px; color: #ccc;"
        )

        self.asset_body.setWordWrap(
            False
        )

        self.asset_body.setAlignment(
            Qt.AlignmentFlag.AlignTop
        )


        main.addWidget(
            self.asset_body
        )


        main.addWidget(
            _divider()
        )


        # --------------------------------------------------------
        # Trend title
        # --------------------------------------------------------

        self.trend_header = QLabel(
            "HOURLY TREND"
        )

        self.trend_header.setStyleSheet(
            "font-size: 12px; color: #888; letter-spacing: 1px;"
        )


        main.addWidget(
            self.trend_header
        )


        # --------------------------------------------------------
        # Trend graph container
        # --------------------------------------------------------

        self.trend_container = QWidget()

        self.trend_layout = QVBoxLayout(
            self.trend_container
        )

        self.trend_layout.setContentsMargins(
            0,
            0,
            0,
            0,
        )

        self.trend_layout.setSpacing(
            4
        )


        self.trend_status = QLabel(
            "No trend data"
        )

        self.trend_status.setStyleSheet(
            "font-size: 12px; color: #777;"
        )


        self.trend_layout.addWidget(
            self.trend_status
        )


        main.addWidget(
            self.trend_container
        )


        main.addWidget(
            _divider()
        )


        # --------------------------------------------------------
        # System
        # --------------------------------------------------------

        sys_header = QLabel(
            "SYSTEM"
        )

        sys_header.setStyleSheet(
            "font-size: 12px; color: #888; letter-spacing: 1px;"
        )


        self.battery = QLabel(
            "Battery SOC: --"
        )

        self.voltage = QLabel(
            "Battery V: --"
        )

        self.tank = QLabel(
            "Tank 1: --"
        )

        self.temp = QLabel(
            "Temp: --"
        )


        main.addWidget(
            sys_header
        )


        for label in (
            self.battery,
            self.voltage,
            self.tank,
            self.temp,
        ):

            label.setStyleSheet(
                "font-size: 15px;"
            )

            main.addWidget(
                label
            )


        main.addWidget(
            _divider()
        )


        # --------------------------------------------------------
        # GPS
        # --------------------------------------------------------

        self.gps = QLabel(
            "--"
        )

        self.gps.setStyleSheet(
            "font-size: 15px; font-weight: bold;"
        )


        main.addWidget(
            self.gps
        )


        main.addStretch()


    # ============================================================
    # GPS
    # ============================================================

    def update_gps(
        self,
        gps,
    ):

        if gps:

            self.gps.setText(
                f"{gps[0]:.6f},  {gps[1]:.6f}"
            )

        else:

            self.gps.setText(
                "--"
            )


    # ============================================================
    # System telemetry
    # ============================================================

    def update_system(
        self,
        data,
    ):

        self.battery.setText(
            f"Battery SOC: "
            f"{data.get('battery_soc', '--')}"
        )

        self.voltage.setText(
            f"Battery V:   "
            f"{data.get('battery_voltage', '--')}"
        )

        self.tank.setText(
            f"Tank 1:      "
            f"{data.get('tank_1_level', '--')}"
        )

        self.temp.setText(
            f"Temp:        "
            f"{data.get('temperature', '--')}"
        )


    # ============================================================
    # Selected asset
    # ============================================================

    def update_asset(
        self,
        asset,
    ):

        self._clear_trend()


        if not asset:

            self.asset_name.setText(
                "None"
            )

            self.asset_body.setText(
                "No asset selected"
            )

            return


        self.asset_name.setText(
            asset.name
        )


        if not asset.sensors:

            self.asset_body.setText(
                "No sensor data"
            )

            return


        lines = []


        for key, value in asset.sensors.items():

            if value is None:

                value = "--"


            lines.append(
                f"{key}: {value}"
            )


        self.asset_body.setText(
            "\n".join(
                lines[:7]
            )
        )


        # --------------------------------------------------------
        # Choose a metric for the first graph
        # --------------------------------------------------------

        metric = self._select_trend_metric(
            asset
        )


        if metric:

            self._load_trend(
                asset.asset_id,
                metric,
            )


    # ============================================================
    # Metric selection
    # ============================================================

    def _select_trend_metric(
        self,
        asset,
    ):

        preferred_metrics = [
            "voltage",
            "level",
            "temperature",
            "wind",
            "flow",
            "soc",
        ]


        available = set(
            asset.sensors.keys()
        )


        for metric in preferred_metrics:

            if metric in available:

                return metric


        return None


    # ============================================================
    # Trend loading
    # ============================================================

    def _load_trend(
        self,
        asset_id,
        metric,
    ):

        self.trend_status.setText(
            f"Loading {metric} history..."
        )


        self._trend_thread = QThread(
            self
        )

        self._trend_worker = TrendWorker(
            self.trend_client,
            asset_id,
            metric,
        )


        self._trend_worker.moveToThread(
            self._trend_thread
        )


        self._trend_thread.started.connect(
            self._trend_worker.run
        )


        self._trend_worker.finished.connect(
            self._trend_loaded
        )


        self._trend_worker.failed.connect(
            self._trend_failed
        )


        self._trend_worker.finished.connect(
            self._trend_thread.quit
        )

        self._trend_worker.failed.connect(
            self._trend_thread.quit
        )


        self._trend_thread.finished.connect(
            self._trend_worker.deleteLater
        )

        self._trend_thread.finished.connect(
            self._trend_thread.deleteLater
        )


        self._trend_thread.start()


    # ============================================================
    # Trend loaded
    # ============================================================

    def _trend_loaded(
        self,
        points,
    ):

        asset = self.state.selected_asset


        if not asset:

            return


        metric = self._select_trend_metric(
            asset
        )


        if not metric:

            return


        minimum, maximum, unit = (
            self._get_scale(
                metric
            )
        )


        graph = MiniTrend(
            minimum,
            maximum,
            unit,
            self
        )


        graph.set_data(
            points
        )


        self.trend_layout.addWidget(
            graph
        )


        self.trend_status.setText(
            f"{metric} — "
            f"{len(points)} hourly points"
        )


    # ============================================================
    # Trend failure
    # ============================================================

    def _trend_failed(
        self,
        error,
    ):

        self.trend_status.setText(
            "Trend unavailable"
        )

        print(
            f"Trend request failed: {error}"
        )


    # ============================================================
    # Clear trend
    # ============================================================

    def _clear_trend(
        self,
    ):

        while (
            self.trend_layout.count()
        ):

            item = (
                self.trend_layout
                .takeAt(0)
            )


            widget = item.widget()


            if widget:

                widget.deleteLater()


        self.trend_status = QLabel(
            "No trend data"
        )

        self.trend_status.setStyleSheet(
            "font-size: 12px; color: #777;"
        )


        self.trend_layout.addWidget(
            self.trend_status
        )


    # ============================================================
    # Engineering ranges
    # ============================================================

    def _get_scale(
        self,
        metric,
    ):

        scales = {

            "voltage": (
                40,
                60,
                "V",
            ),

            "soc": (
                0,
                100,
                "%",
            ),

            "level": (
                0,
                100,
                "%",
            ),

            "temperature": (
                -5,
                50,
                "°C",
            ),

            "wind": (
                0,
                120,
                "km/h",
            ),

            "flow": (
                0,
                20,
                "",
            ),

        }


        return scales.get(
            metric,
            (
                0,
                100,
                "",
            )
        )