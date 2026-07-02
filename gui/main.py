#some notes for updates.
#the icons look pretty grainy.
#theres a concentration of assets around the shed and farm house, a nicer way of viewing these assets so they are easier to select without zooming in would be ideal, perhaps a magnification zone when hovering over an asset? not sure how to solve this issue.
#the cursor is currently a hand which makes selecting assets clunky, change to pointer
#the screen refreshes every minute which is to update the location of vehicles that may be moving. this resets the zoom and deselects any asset that was selected which is not ideal.
#the growing zone asset should also display how many hours ago it was watered, 0 if less than an hour ago.
#vehicles such as ute_1 are not permanently at the farm and will appear at the farm then disappear for up to a month at a time. so it should have a "last seen" which is the last visit it was seen before the current visit. if this time is less than 4 hours then it doesnt count.
#every asset should have a last check-in in hours, if less than an hour it should say <1h which refers to the latest update from that asset, whichever sensor is the latest
#youll have to calc yourself the info for the trends panel, adding rain and human presence days etc over the days. the stuff for the trends panel can all be got from the api to db anyway. the wind trend im not sure if this should be an average or peak or what, but its useful for tracking potential crop damage causes. this info can be put in a yaml so you dont have to calc it every time or if it does have to calc it every time i dont know if thats really slow or if its fine.
#id also like a running list of the latest 20 events with time stamps. and the latest 20 actions like irrigated zone B, pumped water from bore_2 to tank_6, there will be a yaml for both events and actions, or it might be a json
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from registry_loader import load_assets
from app_state import AppState
from scene import build_scene
from view import FarmView
from status_panel import StatusPanel
from trends_panel import TrendsPanel


class MainWindow(QMainWindow):
    def __init__(self, assets):
        super().__init__()
        self.setWindowTitle("Farm UI")

        self.state = AppState()
        self.view = FarmView(build_scene(assets, self.state))
        self.setCentralWidget(self.view)

        self.status_panel = StatusPanel(self, self.state)
        self.trends_panel = TrendsPanel(self, self.state)

        self.view.set_mouse_callback(self.status_panel.update_gps)
        self.state.on_selected_change(self.status_panel.update_asset)
        self.state.on_system_change(self.status_panel.update_system)

        f5 = QShortcut(QKeySequence("F5"), self)
        f5.activated.connect(self._rebuild_scene)

        self._vehicle_timer = QTimer(self)
        self._vehicle_timer.setInterval(60_000)
        self._vehicle_timer.timeout.connect(self._rebuild_scene)
        self._vehicle_timer.start()

        QTimer.singleShot(0, self._post_init)

    def _post_init(self):
        self._resize_panels()
        self.view.align_left()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._resize_panels()
    def _resize_panels(self):
        w = self.width()
        h = self.height()

        status_w = int(w * 0.22)
        trends_w = int(w * 0.29)

        status_h = 380    # asset(6 lines) + system(4) + gps + dividers
        trends_h = 300    # 8 metrics + header + dividers + 1 line spare

        self.trends_panel.setGeometry(
            w - status_w - trends_w,
            h - trends_h,
            trends_w,
            trends_h,
        )

        self.status_panel.setGeometry(
            w - status_w,
            h - status_h,
            status_w,
            status_h,
        )

    def _rebuild_scene(self):
        zoom_index = self.view._zoom_index
        self.state.clear_selection()
        assets = load_assets()
        old_scene = self.view.scene()
        new_scene = build_scene(assets, self.state)
        self.view.setScene(new_scene)
        old_scene.deleteLater()
        self.view._zoom_index = zoom_index
        self.view.resetTransform()
        self.view.scale(
            self.view.ZOOM_STEPS[zoom_index],
            self.view.ZOOM_STEPS[zoom_index],
        )
        self.view.align_left()


def main():
    app = QApplication(sys.argv)
    assets = load_assets()
    window = MainWindow(assets)
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()