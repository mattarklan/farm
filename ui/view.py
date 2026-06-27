from PyQt6.QtWidgets import QGraphicsView
from PyQt6.QtCore import Qt


# Discrete zoom steps: 1.0 is "fit to window" (max zoom-out).
# Values are multipliers on top of the base fit scale.
ZOOM_STEPS = [1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0]


class FarmView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)

        self._base_scale = 1.0      # set by fit_to_window()
        self._zoom_index = 0        # index into ZOOM_STEPS

        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setViewportUpdateMode(
            QGraphicsView.ViewportUpdateMode.FullViewportUpdate
        )
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    # ------------------------------------------------------------------
    # Call once after show() to lock the "fit" level as zoom step 0
    # ------------------------------------------------------------------
    def fit_to_window(self):
        rect = self.scene().sceneRect()
        vw, vh = self.viewport().width(), self.viewport().height()

        scale_x = vw / rect.width()
        scale_y = vh / rect.height()
        # Use the larger scale so the background fills the window edge-to-edge
        self._base_scale = max(scale_x, scale_y)

        self.resetTransform()
        self.scale(self._base_scale, self._base_scale)
        self._zoom_index = 0

        self._notify_scene_zoom()

    # ------------------------------------------------------------------
    # Scroll wheel → step through discrete zoom levels
    # ------------------------------------------------------------------
    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            new_index = min(self._zoom_index + 1, len(ZOOM_STEPS) - 1)
        else:
            new_index = max(self._zoom_index - 1, 0)

        if new_index == self._zoom_index:
            return

        old_step = ZOOM_STEPS[self._zoom_index]
        new_step = ZOOM_STEPS[new_index]
        factor = new_step / old_step

        self._zoom_index = new_index
        self.scale(factor, factor)

        self._notify_scene_zoom()

    # ------------------------------------------------------------------
    # Tell every FixedSizeItem in the scene what the current scale is
    # so they can paint themselves at constant pixel size.
    # ------------------------------------------------------------------
    def _notify_scene_zoom(self):
        current_scale = self._base_scale * ZOOM_STEPS[self._zoom_index]
        for item in self.scene().items():
            if hasattr(item, "on_zoom_changed"):
                item.on_zoom_changed(current_scale)

    # ------------------------------------------------------------------
    # Keep background filling the window when the user resizes
    # ------------------------------------------------------------------
    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Only refit when fully zoomed out; otherwise keep the user's zoom
        if self._zoom_index == 0:
            self.fit_to_window()
