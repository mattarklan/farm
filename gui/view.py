from PyQt6.QtWidgets import QGraphicsView
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QBrush, QCursor


class FarmView(QGraphicsView):

    ZOOM_STEPS = [
        0.235,
        0.40,
        0.50,
        0.65,
        0.80,
        1.0,
    ]

    START_INDEX = 0


    def __init__(
        self,
        scene
    ):

        super().__init__(
            scene
        )


        self._zoom_index = self.START_INDEX

        self._mouse_callback = None


        self.setAlignment(
            Qt.AlignmentFlag.AlignLeft |
            Qt.AlignmentFlag.AlignTop
        )


        self.setMouseTracking(
            True
        )


        # normal pointer instead of hand
        self.setCursor(
            QCursor(
                Qt.CursorShape.ArrowCursor
            )
        )


        self.setDragMode(
            QGraphicsView.DragMode.NoDrag
        )


        self.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )

        self.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )


        self.setBackgroundBrush(
            QBrush(
                QColor("#1a1a1a")
            )
        )


        self.resetTransform()

        self.scale(
            self.ZOOM_STEPS[self._zoom_index],
            self.ZOOM_STEPS[self._zoom_index]
        )



    # --------------------------------------------------------
    # Scene switching
    # --------------------------------------------------------

    def set_scene(
        self,
        scene
    ):

        self.setScene(
            scene
        )

        self.resetTransform()

        self.scale(
            self.ZOOM_STEPS[self._zoom_index],
            self.ZOOM_STEPS[self._zoom_index]
        )



    # --------------------------------------------------------
    # Map positioning
    # --------------------------------------------------------

    def align_left(self):

        self.horizontalScrollBar().setValue(
            self.horizontalScrollBar().minimum()
        )

        self.verticalScrollBar().setValue(
            self.verticalScrollBar().minimum()
        )



    # --------------------------------------------------------
    # SCADA positioning
    # --------------------------------------------------------

    def center_scene(self):

        scene = self.scene()

        if not scene:
            return


        self.fitInView(
            scene.sceneRect(),
            Qt.AspectRatioMode.KeepAspectRatio
        )


    # --------------------------------------------------------
    # Mouse callback
    # --------------------------------------------------------

    def set_mouse_callback(
        self,
        fn
    ):

        self._mouse_callback = fn



    def mouseMoveEvent(
        self,
        event
    ):

        scene_pos = self.mapToScene(
            event.position().toPoint()
        )

        scene = self.scene()

        gps = None

        if hasattr(
            scene,
            "projector"
        ):

            gps = scene.projector.to_gps(
                scene_pos.x(),
                scene_pos.y()
            )


        if self._mouse_callback:

            self._mouse_callback(
                gps
            )


        super().mouseMoveEvent(
            event
        )



    # --------------------------------------------------------
    # Zoom
    # --------------------------------------------------------

    def wheelEvent(
        self,
        event
    ):

        delta = event.angleDelta().y()


        if delta > 0:

            new_index = min(
                self._zoom_index + 1,
                len(self.ZOOM_STEPS) - 1
            )

        else:

            new_index = max(
                self._zoom_index - 1,
                0
            )


        if new_index == self._zoom_index:

            event.accept()

            return



        old = self.ZOOM_STEPS[
            self._zoom_index
        ]

        new = self.ZOOM_STEPS[
            new_index
        ]


        self._zoom_index = new_index


        self.scale(
            new / old,
            new / old
        )


        event.accept()



    def resizeEvent(
        self,
        event
    ):

        super().resizeEvent(
            event
        )