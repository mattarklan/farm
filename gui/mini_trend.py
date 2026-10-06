from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QColor, QFont
from PyQt6.QtCore import Qt


class MiniTrend(QWidget):

    def __init__(
        self,
        minimum,
        maximum,
        unit="",
        parent=None
    ):

        super().__init__(parent)

        self.minimum = minimum
        self.maximum = maximum
        self.unit = unit

        self.points = []

        self.setMinimumHeight(90)
        self.setMaximumHeight(110)



    def set_data(
        self,
        points
    ):

        """
        points should be a list of dictionaries:

        [
            {
                "timestamp": 1784379600,
                "value": 51.10
            },
            ...
        ]
        """

        self.points = points or []

        self.update()



    def paintEvent(
        self,
        event
    ):

        painter = QPainter(self)

        painter.setRenderHint(
            QPainter.RenderHint.Antialiasing,
            False
        )


        width = self.width()
        height = self.height()


        # ----------------------------------------------------
        # Margins
        # ----------------------------------------------------

        left = 38
        right = 8
        top = 8
        bottom = 18


        graph_width = width - left - right
        graph_height = height - top - bottom


        # ----------------------------------------------------
        # Background
        # ----------------------------------------------------

        painter.fillRect(
            0,
            0,
            width,
            height,
            QColor("#202020")
        )


        # ----------------------------------------------------
        # Border
        # ----------------------------------------------------

        painter.setPen(
            QPen(
                QColor("#444444"),
                1
            )
        )

        painter.drawRect(
            left,
            top,
            graph_width,
            graph_height
        )


        # ----------------------------------------------------
        # Scale labels
        # ----------------------------------------------------

        painter.setPen(
            QColor("#888888")
        )

        painter.setFont(
            QFont(
                "Arial",
                8
            )
        )


        painter.drawText(
            0,
            top + 8,
            f"{self.maximum:g}"
        )


        painter.drawText(
            0,
            top + graph_height,
            f"{self.minimum:g}"
        )


        # ----------------------------------------------------
        # No data
        # ----------------------------------------------------

        if not self.points:

            painter.setPen(
                QColor("#666666")
            )

            painter.drawText(
                left + 5,
                top + graph_height // 2,
                "No history"
            )

            return


        # ----------------------------------------------------
        # Fixed engineering scale
        # ----------------------------------------------------

        value_range = (
            self.maximum -
            self.minimum
        )


        if value_range <= 0:

            return


        # ----------------------------------------------------
        # Draw points
        # ----------------------------------------------------

        count = len(
            self.points
        )


        for i, point in enumerate(
            self.points
        ):

            value = point.get(
                "value"
            )


            if value is None:

                continue


            # Clamp values to graph range

            value = max(
                self.minimum,
                min(
                    self.maximum,
                    value
                )
            )


            # X position

            if count == 1:

                x = left + (
                    graph_width // 2
                )

            else:

                x = left + int(
                    i *
                    graph_width /
                    (count - 1)
                )


            # Y position

            normalized = (
                value -
                self.minimum
            ) / value_range


            y = top + int(
                graph_height *
                (1 - normalized)
            )


            # 2x2 pixel dot

            painter.fillRect(
                x,
                y,
                2,
                2,
                QColor("#FFFFFF")
            )


        # ----------------------------------------------------
        # Time labels
        # ----------------------------------------------------

        painter.setPen(
            QColor("#777777")
        )


        painter.drawText(
            left,
            height - 3,
            "oldest"
        )


        painter.drawText(
            width - 32,
            height - 3,
            "now"
        )


        painter.end()