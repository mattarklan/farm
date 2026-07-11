from PyQt6.QtWidgets import QGraphicsScene, QGraphicsPixmapItem
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt

from .assets import AssetItem


SCENE_W = 4000
SCENE_H = 3000



# ------------------------------------------------------------
# GPS PROJECTOR
# ------------------------------------------------------------

class SimpleMapProjector:

    def __init__(self):

        self.min_lat = -31.9510
        self.max_lat = -31.9500

        self.min_lon = 115.8600
        self.max_lon = 115.8610



    def to_scene(
        self,
        lat,
        lon
    ):

        x = (
            lon - self.min_lon
        ) / (
            self.max_lon - self.min_lon
        ) * SCENE_W


        y = (
            lat - self.min_lat
        ) / (
            self.max_lat - self.min_lat
        ) * SCENE_H


        return x, y



    def to_gps(
        self,
        x,
        y
    ):

        lon = (
            x / SCENE_W
        ) * (
            self.max_lon - self.min_lon
        ) + self.min_lon


        lat = (
            y / SCENE_H
        ) * (
            self.max_lat - self.min_lat
        ) + self.min_lat


        return lat, lon




# ------------------------------------------------------------
# SCENE
# ------------------------------------------------------------

class FarmScene(QGraphicsScene):

    def __init__(
        self,
        app_state
    ):

        super().__init__()

        self.app_state = app_state

        self.projector = SimpleMapProjector()

        self.setSceneRect(
            0,
            0,
            SCENE_W,
            SCENE_H
        )




# ------------------------------------------------------------
# BACKGROUND
# ------------------------------------------------------------

def load_background():

    pix = QPixmap(
        "farm/gui/farm.png"
    )


    if pix.isNull():

        raise RuntimeError(
            "Missing farm.png"
        )


    return pix.scaled(
        SCENE_W,
        SCENE_H,
        Qt.AspectRatioMode.IgnoreAspectRatio,
        Qt.TransformationMode.SmoothTransformation
    )




# ------------------------------------------------------------
# BUILD
# ------------------------------------------------------------

def build_scene(
    assets,
    app_state
):

    scene = FarmScene(
        app_state
    )


    # background

    bg = QGraphicsPixmapItem(
        load_background()
    )

    bg.setZValue(
        -100
    )

    scene.addItem(
        bg
    )



    # assets

    for asset_state in assets:


        app_state.register_asset(
            asset_state
        )


        item = AssetItem(
            asset_state
        )


        if asset_state.gps:

            lat, lon = asset_state.gps


            x, y = scene.projector.to_scene(
                lat,
                lon
            )


            item.setPos(
                x,
                y
            )



        item.on_click = (

            lambda s=asset_state:
            scene.app_state.set_selected_asset(
                s
            )

        )


        scene.addItem(
            item
        )


    return scene