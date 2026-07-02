from PyQt6.QtWidgets import QGraphicsItem
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtCore import QRectF, Qt

ICON_SIZE = 80

ICON_MAP = {
    "tank":     "icons/tank.png",
    "bore":     "icons/bore.png",
    "pump":     "icons/tap.png",
    "vehicle":  "icons/vehicle.png",
    "tractor":  "icons/tractor.png",
    "power":    "icons/power.png",
    "crop":     "icons/crop.png",
}
DEFAULT_ICON = "icons/default.png"


def load_icon(path):
    pix = QPixmap(path)
    if pix.isNull():
        pix = QPixmap(ICON_SIZE, ICON_SIZE)
        pix.fill(QColor("#FF5555"))
    return pix.scaled(
        ICON_SIZE,
        ICON_SIZE,
        Qt.AspectRatioMode.KeepAspectRatio,
        Qt.TransformationMode.SmoothTransformation,
    )


class AssetItem(QGraphicsItem):
    def __init__(self, asset_state):
        super().__init__()
        self.state = asset_state
        self.on_click = None

        icon_path = ICON_MAP.get(asset_state.type, DEFAULT_ICON)
        self._icon = load_icon(icon_path)

        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)

    def boundingRect(self):
        return QRectF(-ICON_SIZE / 2, -ICON_SIZE / 2, ICON_SIZE, ICON_SIZE)

    def paint(self, painter, option, widget=None):
        painter.save()
        painter.drawPixmap(-ICON_SIZE // 2, -ICON_SIZE // 2, self._icon)
        if self.isSelected():
            painter.setPen(QColor("#FFD400"))
            painter.drawRect(-ICON_SIZE // 2, -ICON_SIZE // 2, ICON_SIZE, ICON_SIZE)
        painter.restore()

    def mousePressEvent(self, event):
        if self.on_click:
            self.on_click()
        super().mousePressEvent(event)