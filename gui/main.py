import sys

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QToolBar,
)

from PyQt6.QtGui import QAction
from PyQt6.QtCore import QTimer

from .app_state import AppState
from .scene import build_scene
from .scada_scene import SCADAScene
from .view import FarmView
from .status_panel import StatusPanel
from .trends_panel import TrendsPanel
from .asset_adapter import registry_to_assets

from farm.config.loader import load_registry
from farm.core.api_client import IngestionAPIClient



class MainWindow(QMainWindow):

    def __init__(
        self,
        assets
    ):

        super().__init__()

        self.setWindowTitle(
            "Farm UI"
        )


        # ----------------------------------------------------
        # Shared application state
        # ----------------------------------------------------

        self.state = AppState()


        # ----------------------------------------------------
        # Build scenes once
        # ----------------------------------------------------

        self.map_scene = build_scene(
            assets,
            self.state,
        )

        self.scada_scene = SCADAScene(
            self.state
        )


        self.view = FarmView(
            self.map_scene
        )

        self.setCentralWidget(
            self.view
        )


        # ----------------------------------------------------
        # API
        # ----------------------------------------------------

        self.api = IngestionAPIClient(
            "http://127.0.0.1:8000"
        )


        # ----------------------------------------------------
        # Panels
        # ----------------------------------------------------

        self.status_panel = StatusPanel(
            self,
            self.state
        )

        self.trends_panel = TrendsPanel(
            self,
            self.state
        )


        self.state.on_selected_change(
            self.status_panel.update_asset
        )

        self.state.on_system_change(
            self.status_panel.update_system
        )


        # ----------------------------------------------------
        # Toolbar
        # ----------------------------------------------------

        toolbar = QToolBar(
            "Views"
        )

        self.addToolBar(
            toolbar
        )


        map_action = QAction(
            "Map",
            self
        )

        process_action = QAction(
            "Process",
            self
        )


        map_action.triggered.connect(
            self.show_map
        )

        process_action.triggered.connect(
            self.show_scada
        )


        toolbar.addAction(
            map_action
        )

        toolbar.addAction(
            process_action
        )


        # ----------------------------------------------------
        # Telemetry refresh
        # ----------------------------------------------------

        self._telemetry_timer = QTimer(
            self
        )

        self._telemetry_timer.setInterval(
            10_000
        )

        self._telemetry_timer.timeout.connect(
            self.refresh_telemetry
        )

        self._telemetry_timer.start()


        # Initial data load
        QTimer.singleShot(
            0,
            self.refresh_telemetry
        )


        QTimer.singleShot(
            0,
            self._post_init
        )



    # --------------------------------------------------------
    # Views
    # --------------------------------------------------------

    def show_map(self):

        self.view.set_scene(
            self.map_scene
        )

        self.view.align_left()



    def show_scada(self):

        self.view.set_scene(
            self.scada_scene
        )

        self.view.center_scene()



    # --------------------------------------------------------
    # API telemetry
    # --------------------------------------------------------

    def refresh_telemetry(self):

        try:

            response = self.api.get_latest_state()


        except Exception as e:

            print(
                "Telemetry update failed:",
                e
            )

            return



        for asset in response.get(
            "assets",
            []
        ):

            self.state.update_asset_sensors(
                asset["asset_id"],
                asset.get(
                    "metrics",
                    {}
                ),
                asset.get(
                    "ts"
                ),
            )



    # --------------------------------------------------------
    # Layout
    # --------------------------------------------------------

    def _post_init(self):

        self._resize_panels()

        self.view.align_left()



    def resizeEvent(
        self,
        event
    ):

        super().resizeEvent(
            event
        )

        self._resize_panels()



    def _resize_panels(self):

        w = self.width()
        h = self.height()


        status_w = int(
            w * 0.22
        )

        trends_w = int(
            w * 0.29
        )


        self.trends_panel.setGeometry(
            w - status_w - trends_w,
            h - 300,
            trends_w,
            300,
        )


        self.status_panel.setGeometry(
            w - status_w,
            h - 380,
            status_w,
            380,
        )





def main():

    app = QApplication(
        sys.argv
    )


    registry = load_registry()

    assets = registry_to_assets(
        registry
    )


    window = MainWindow(
        assets
    )

    window.showMaximized()


    sys.exit(
        app.exec()
    )



if __name__ == "__main__":

    main()