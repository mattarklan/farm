from PyQt6.QtWidgets import QGraphicsView
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QBrush


class FarmView(QGraphicsView):
    ZOOM_STEPS = [0.235, 0.40, 0.50, 0.65, 0.80, 1.0]
    START_INDEX = 0  # 0.65x

    def __init__(self, scene):
        super().__init__(scene)
        self._zoom_index = self.START_INDEX
        self._mouse_callback = None
        self.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.setMouseTracking(True)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        # dark background instead of white
        self.setBackgroundBrush(QBrush(QColor("#1a1a1a")))

        self.resetTransform()
        self.scale(self.ZOOM_STEPS[self._zoom_index], self.ZOOM_STEPS[self._zoom_index])

    def set_mouse_callback(self, fn):
        self._mouse_callback = fn

    def align_left(self):
        """Call after window is shown — scrolls scene to left edge."""
        self.horizontalScrollBar().setValue(self.horizontalScrollBar().minimum())
        self.verticalScrollBar().setValue(self.verticalScrollBar().minimum())

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        new_index = min(self._zoom_index + 1, len(self.ZOOM_STEPS) - 1) if delta > 0 \
                    else max(self._zoom_index - 1, 0)

        if new_index == self._zoom_index:
            event.accept()
            return

        old = self.ZOOM_STEPS[self._zoom_index]
        new = self.ZOOM_STEPS[new_index]
        self._zoom_index = new_index
        self.scale(new / old, new / old)
        event.accept()

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.position().toPoint())
        scene = self.scene()
        gps = None
        if hasattr(scene, "projector"):
            gps = scene.projector.to_gps(scene_pos.x(), scene_pos.y())
        if self._mouse_callback:
            self._mouse_callback(gps)
        super().mouseMoveEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)