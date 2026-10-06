from PyQt6.QtWidgets import (
    QGraphicsScene,
    QGraphicsTextItem,
    QGraphicsRectItem,
)

from PyQt6.QtGui import (
    QPen,
    QColor,
    QFont,
)

from PyQt6.QtCore import QRectF

from .assets import AssetItem


SCENE_W = 4000
SCENE_H = 3000


PIPE_IDLE = QColor("#FFFFFF")
PIPE_ACTIVE = QColor("#2196F3")

ACTUATOR_OFF = QColor("#555555")
ACTUATOR_ON = QColor("#2196F3")


ICON_SCALE = 1
TEXT_SIZE = 12

PIPE_WIDTH = 8

ACTUATOR_SIZE = 40



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
        self.labels = {}

        self.connections = {}

        self.actuators = {}


        self.build()


        self.app_state.on_asset_change(
            self.update_asset
        )



    # --------------------------------------------------------
    # Build
    # --------------------------------------------------------

    def build(self):

        assets = self.app_state.get_all_assets()


        positions = self.calculate_positions(
            assets
        )


        for asset in assets:

            item = AssetItem(
                asset
            )

            item.setScale(
                ICON_SCALE
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


            self.add_label(
                asset
            )


        self.build_connections(
            assets
        )



    # --------------------------------------------------------
    # Automatic graph layout
    # --------------------------------------------------------

    def calculate_positions(
        self,
        assets
    ):

        depth_cache = {}


        def depth(asset):

            if asset.asset_id in depth_cache:
                return depth_cache[
                    asset.asset_id
                ]


            if not asset.dependencies:
                depth_cache[
                    asset.asset_id
                ] = 0

                return 0


            values = []

            for dep in asset.dependencies:

                parent = next(
                    (
                        a for a in assets
                        if a.asset_id == dep
                    ),
                    None
                )

                if parent:

                    values.append(
                        depth(parent)
                    )


            result = (
                max(values) + 1
                if values
                else 0
            )


            depth_cache[
                asset.asset_id
            ] = result


            return result



        layers = {}


        for asset in assets:

            d = depth(
                asset
            )

            layers.setdefault(
                d,
                []
            ).append(
                asset
            )



        positions = {}


        max_layer = max(
            layers.keys()
        ) if layers else 0



        for layer, items in layers.items():

            spacing = (
                SCENE_W //
                (len(items)+1)
            )


            y = (
                300
                +
                layer *
                (
                    SCENE_H //
                    (max_layer+2)
                )
            )


            for index, asset in enumerate(
                items,
                start=1
            ):

                positions[
                    asset.asset_id
                ] = (
                    spacing * index,
                    y
                )


        return positions



    # --------------------------------------------------------
    # Labels
    # --------------------------------------------------------

    def add_label(
        self,
        asset
    ):

        text = QGraphicsTextItem()

        text.setDefaultTextColor(
            QColor("#FFFFFF")
        )

        text.setFont(
            QFont(
                "Arial",
                TEXT_SIZE
            )
        )

        text.setPlainText(
            asset.name
        )


        item = self.asset_items[
            asset.asset_id
        ]


        text.setPos(
            item.x()-50,
            item.y()+70
        )


        self.addItem(
            text
        )


        self.labels[
            asset.asset_id
        ] = text



    # --------------------------------------------------------
    # Connections
    # --------------------------------------------------------

    def build_connections(
        self,
        assets
    ):

        lookup = {
            a.asset_id:a
            for a in assets
        }


        for asset in assets:

            for dependency in asset.dependencies:


                if dependency not in self.asset_items:
                    continue


                source = self.asset_items[
                    dependency
                ]

                target = self.asset_items[
                    asset.asset_id
                ]


                line = self.addLine(
                    source.x(),
                    source.y(),
                    target.x(),
                    target.y(),
                    QPen(
                        PIPE_IDLE,
                        PIPE_WIDTH
                    )
                )


                line.setZValue(
                    -1
                )


                key = (
                    dependency,
                    asset.asset_id
                )


                self.connections[
                    key
                ] = line



                self.create_actuator(
                    key,
                    source,
                    target
                )



    # --------------------------------------------------------
    # Connection actuator
    # --------------------------------------------------------

    def create_actuator(
        self,
        key,
        source,
        target
    ):

        x = (
            source.x()
            +
            target.x()
        ) / 2


        y = (
            source.y()
            +
            target.y()
        ) / 2



        block = QGraphicsRectItem(
            QRectF(
                x - ACTUATOR_SIZE/2,
                y - ACTUATOR_SIZE/2,
                ACTUATOR_SIZE,
                ACTUATOR_SIZE
            )
        )


        block.setBrush(
            ACTUATOR_OFF
        )


        block.setPen(
            QPen(
                QColor("#FFFFFF"),
                1
            )
        )


        self.addItem(
            block
        )


        self.actuators[
            key
        ] = block



    # --------------------------------------------------------
    # Telemetry update
    # --------------------------------------------------------

    def update_asset(
        self,
        asset
    ):

        if asset.asset_id in self.labels:

            text = asset.name


            for key,value in asset.sensors.items():

                if value is not None:

                    text += (
                        f"\n{key}: {value}"
                    )


            self.labels[
                asset.asset_id
            ].setPlainText(
                text
            )



        self.update_connections()



    # --------------------------------------------------------
    # Pipe state
    # --------------------------------------------------------

    def update_connections(
        self
    ):

        assets = {
            a.asset_id:a
            for a in self.app_state.get_all_assets()
        }


        for (
            source,
            target
        ), line in self.connections.items():


            asset = assets.get(
                source
            )


            active = (
                asset is not None
                and self.asset_active(asset)
            )


            line.setPen(
                QPen(
                    PIPE_ACTIVE
                    if active
                    else PIPE_IDLE,
                    PIPE_WIDTH
                )
            )


            if (
                source,
                target
            ) in self.actuators:

                self.actuators[
                    (
                        source,
                        target
                    )
                ].setBrush(
                    ACTUATOR_ON
                    if active
                    else ACTUATOR_OFF
                )



    def asset_active(
        self,
        asset
    ):

        for key,value in asset.sensors.items():

            if value is None:
                continue


            key = key.lower()


            if any(
                x in key
                for x in [
                    "flow",
                    "current",
                    "power",
                    "running",
                    "state"
                ]
            ):

                try:

                    if float(value) > 0.5:
                        return True

                except:

                    if str(value).lower() in (
                        "on",
                        "true",
                        "running"
                    ):
                        return True


        return False