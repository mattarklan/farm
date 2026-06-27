import sys
from PyQt6.QtWidgets import QApplication

from scene import build_scene
from view import FarmView
from detail_panel import DetailPanel
from registry_loader import AssetRegistry


# ----------------------------------------------------------------------
# MAIN ENTRY
# ----------------------------------------------------------------------

def main():
    app = QApplication(sys.argv)

    # Load YAML-backed assets
    registry = AssetRegistry("../config/registry/water.yaml")
    assets = registry.load()

    # Build scene
    scene = build_scene(assets)

    # View
    view = FarmView(scene)
    view.setWindowTitle("Farm Viewer")

    # Panel
    panel = DetailPanel(view)
    scene.asset_click_callback = panel.show_asset

    view.showMaximized()
    view.fit_to_window()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()