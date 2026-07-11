from PyQt6.QtWidgets import QGraphicsScene, QGraphicsLineItem
from PyQt6.QtGui import QPen, QColor
from PyQt6.QtCore import Qt

from .assets import AssetItem


SCENE_W = 4000
SCENE_H = 3000


PIPE_IDLE = QColor("#FFFFFF")
PIPE_ACTIVE = QColor("#2196F3")


class SCADAScene(QGraphicsScene):

    def __init__(
        self,
        app_state
    ):

        super().__init__()

        self.app_state = app_state

        self.setSceneRect(
            0,
            0,
            SCENE_W,
            SCENE_H
        )


        self.asset_items = {}

        self.pipe_items = {}


        self.build()


        self.app_state.on_asset_change(
            self.update_asset
        )



    # --------------------------------------------------------
    # Build
    # --------------------------------------------------------

    def build(self):

        positions = self.generate_layout()


        for asset in self.app_state.get_all_assets():

            if asset.asset_id not in positions:
                continue


            item = AssetItem(
                asset
            )


            x, y = positions[
                asset.asset_id
            ]

            item.setPos(
                x,
                y
            )


            item.on_click = (
                lambda a=asset:
                self.app_state.set_selected_asset(a)
            )


            self.addItem(
                item
            )


            self.asset_items[
                asset.asset_id
            ] = item



        self.build_pipes()



    # --------------------------------------------------------
    # Layout
    # --------------------------------------------------------

    def generate_layout(self):

        layout = {}


        humans = []
        weather = []
        bores = []
        tanks = []
        zones = []


        for asset in self.app_state.get_all_assets():

            t = asset.type


            if t == "human":
                humans.append(asset)

            elif t in (
                "weather",
                "weather_station"
            ):
                weather.append(asset)

            elif t == "bore":
                bores.append(asset)

            elif t == "tank":
                tanks.append(asset)

            elif t in (
                "crop",
                "zone",
                "growing_zone"
            ):
                zones.append(asset)



        def place_row(
            assets,
            y
        ):

            if not assets:
                return


            spacing = (
                SCENE_W //
                (len(assets) + 1)
            )


            for i, asset in enumerate(
                assets,
                start=1
            ):

                layout[
                    asset.asset_id
                ] = (
                    spacing * i,
                    y
                )


        place_row(
            humans,
            250
        )

        place_row(
            weather + bores,
            750
        )

        place_row(
            tanks,
            1450
        )

        place_row(
            zones,
            2250
        )


        return layout



    # --------------------------------------------------------
    # Pipes
    # --------------------------------------------------------

    def build_pipes(self):

        for asset in self.app_state.get_all_assets():

            child = asset.asset_id


            for parent in self.get_dependencies(
                child
            ):

                if (
                    parent not in self.asset_items
                    or child not in self.asset_items
                ):
                    continue


                a = self.asset_items[parent]
                b = self.asset_items[child]


                line = QGraphicsLineItem(
                    a.x(),
                    a.y(),
                    b.x(),
                    b.y()
                )


                line.setPen(
                    QPen(
                        PIPE_IDLE,
                        8
                    )
                )


                line.setZValue(
                    -1
                )


                self.addItem(
                    line
                )


                self.pipe_items[
                    (parent, child)
                ] = line



    # --------------------------------------------------------
    # Dependencies
    # --------------------------------------------------------

    def get_dependencies(
        self,
        asset_id
    ):

        asset = self.app_state.get_asset(
            asset_id
        )

        if not asset:
            return []


        # AppState currently stores only GUI state.
        # Dependencies will be attached here next.
        return getattr(
            asset,
            "dependencies",
            []
        )



    # --------------------------------------------------------
    # Telemetry updates
    # --------------------------------------------------------

    def update_asset(
        self,
        asset
    ):

        #
        # Next step:
        # change pipe colours based on flow
        #
        pass