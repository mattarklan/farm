from PyQt6.QtWidgets import QGraphicsScene, QGraphicsPixmapItem
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt

from assets import AssetItem

SCENE_W, SCENE_H = 4000, 3000


# ----------------------------------------------------------------------
# GPS projector (temporary linear mapping)
# ----------------------------------------------------------------------

class SimpleMapProjector:
    def __init__(self):
        self.min_lat = -31.9510
        self.max_lat = -31.9500
        self.min_lon = 115.8600
        self.max_lon = 115.8610

    def to_scene(self, lat: float, lon: float):
        x = (lon - self.min_lon) / (self.max_lon - self.min_lon) * SCENE_W
        y = (lat - self.min_lat) / (self.max_lat - self.min_lat) * SCENE_H
        return x, y


# ----------------------------------------------------------------------
# Scene
# ----------------------------------------------------------------------

class FarmScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.asset_click_callback = None
        self.projector = SimpleMapProjector()

    def asset_selected(self, asset):
        if self.asset_click_callback:
            self.asset_click_callback(asset)


# ----------------------------------------------------------------------
# Background loader
# ----------------------------------------------------------------------

def _load_background():
    pix = QPixmap("farm.png")
    if pix.isNull():
        raise RuntimeError("Missing farm.png")

    return pix.scaled(
        SCENE_W,
        SCENE_H,
        Qt.AspectRatioMode.IgnoreAspectRatio,
        Qt.TransformationMode.SmoothTransformation
    )


# ----------------------------------------------------------------------
# Scene builder
# ----------------------------------------------------------------------

def build_scene(assets: list):
    scene = FarmScene()
    scene.setSceneRect(0, 0, SCENE_W, SCENE_H)

    # background
    bg = QGraphicsPixmapItem(_load_background())
    bg.setZValue(0)
    scene.addItem(bg)

    # assets
    for asset in assets:
        item = AssetItem(asset)

        gps = getattr(asset, "gps", None)
        if gps:
            lat, lon = gps
            x, y = scene.projector.to_scene(lat, lon)
            item.setPos(x, y)

        scene.addItem(item)

    return scene