from PyQt6.QtWidgets import QGraphicsItem
from PyQt6.QtGui import QPixmap, QPainter, QColor
from PyQt6.QtCore import Qt, QRectF

ICON_PX = 40


# ----------------------------------------------------------------------
# Single icon for water domain (for now)
# ----------------------------------------------------------------------

TANK_ICON_PATH = "icons/tank.png"


def load_icon():
    pix = QPixmap(TANK_ICON_PATH)

    if pix.isNull():
        pix = QPixmap(ICON_PX, ICON_PX)
        pix.fill(QColor("#FF6B6B"))  # visible failure

    return pix.scaled(
        ICON_PX,
        ICON_PX,
        Qt.AspectRatioMode.KeepAspectRatio,
        Qt.TransformationMode.SmoothTransformation,
    )


# ----------------------------------------------------------------------
# Asset renderer
# ----------------------------------------------------------------------

class AssetItem(QGraphicsItem):
    """
    Water domain renderer only.

    No type logic.
    No icon mapping.
    No guessing.
    """

    def __init__(self, asset):
        super().__init__()

        self.asset = asset
        self.asset_id = asset.id
        self.label = asset.name

        self._pixmap = load_icon()
        self._scale = 1.0

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setZValue(10)

    # --------------------------------------------------------------
    # Zoom handling
    # --------------------------------------------------------------

    def on_zoom_changed(self, scene_scale: float):
        self._scale = scene_scale
        self.prepareGeometryChange()
        self.update()

    # --------------------------------------------------------------
    # Geometry
    # --------------------------------------------------------------

    def boundingRect(self) -> QRectF:
        size = ICON_PX / self._scale
        return QRectF(0, 0, size, size)

    # --------------------------------------------------------------
    # Paint
    # --------------------------------------------------------------

    def paint(self, painter: QPainter, option, widget=None):
        painter.save()

        painter.scale(1.0 / self._scale, 1.0 / self._scale)
        painter.drawPixmap(0, 0, self._pixmap)

        if self.isSelected():
            painter.setPen(QColor("#FFD400"))
            painter.drawRect(0, 0, ICON_PX, ICON_PX)

        painter.restore()

    # --------------------------------------------------------------
    # Interaction
    # --------------------------------------------------------------

    def mousePressEvent(self, event):
        super().mousePressEvent(event)

        scene = self.scene()
        if scene and hasattr(scene, "asset_click_callback"):
            scene.asset_click_callback(self.asset)