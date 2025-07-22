from PySide6.QtWidgets import QGraphicsWidget
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PySide6.QtCore import Qt, QRectF, QPointF

from utils import frames_to_timecode
from model.data import ClipData, TrackData


class TimeLabelItem(QGraphicsWidget):
    def __init__(self, initial_frame: int, theme: dict, parent=None):
        super().__init__(parent)
        self.frame = initial_frame
        self.theme = theme
        self.setZValue(100)

    def updateTime(self, frame: int):
        self.frame = frame
        self.update()

    def paint(self, painter: QPainter, option, widget):
        fps = self.theme.get('FPS', 24)
        text = frames_to_timecode(self.frame, fps)
        font = QFont(
            self.theme.get('timeLabel_font_family', 'Monospace'),
            self.theme.get('timeLabel_font_size', 10)
        )
        painter.setFont(font)
        painter.setPen(QPen(QColor(self.theme['timeLabel_text'])))
        painter.drawText(
            QRectF(0, 0, self.boundingRect().width(), self.boundingRect().height()),
            Qt.AlignLeft | Qt.AlignVCenter,
            text
        )

    def boundingRect(self):
        width = self.theme.get('LEFT_MARGIN', 200)
        height = self.theme.get('timeLabel_height', 20)
        return QRectF(0, 0, width, height)


class RulerItem(QGraphicsWidget):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setZValue(50)

    def paint(self, painter: QPainter, option, widget):
        scene = self.scene()
        view = scene.views()[0] if scene and scene.views() else None
        if not view:
            return

        fps = self.theme.get('FPS', 24)
        pixpf = self.theme['BASE_PIXELS_PER_FRAME'] * view.h_zoom
        left = self.theme['LEFT_MARGIN']
        end_frame = max(view.end_frame, 1)
        max_x = left + end_frame * pixpf

        # Base line
        painter.setPen(QPen(QColor(self.theme['ruler_color']), 1))
        painter.drawLine(left, 0, max_x, 0)

        # Major ticks and labels every `fps` frames (1 second)
        interval = fps
        font = QFont(
            self.theme.get('ruler_font_family', 'Monospace'),
            self.theme.get('ruler_font_size', 8)
        )
        painter.setFont(font)
        tick_len = self.theme.get('tick_length', 5)
        label_offset = self.theme.get('ruler_label_offset', 12)
        for f in range(0, end_frame + 1, interval):
            x = left + f * pixpf
            # Tick line
            painter.drawLine(x, 0, x, tick_len)
            # Timecode label
            text = frames_to_timecode(f, fps)
            painter.drawText(QPointF(x + 2, tick_len + label_offset), text)

    def boundingRect(self):
        width = self.theme.get('LEFT_MARGIN', 200) + self.theme['BASE_PIXELS_PER_FRAME'] * 100 * self.theme.get('initial_h_zoom', 1.0)
        height = self.theme.get('ruler_height', 20)
        return QRectF(0, 0, width, height)


class PlayheadTriangleItem(QGraphicsWidget):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setZValue(200)

    def paint(self, painter: QPainter, option, widget):
        size = self.theme.get('playhead_triangle_size', 10)
        half = size / 2
        points = [QPointF(0, 0), QPointF(-half, size), QPointF(half, size)]
        painter.setBrush(QBrush(QColor(self.theme['playhead_color'])))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(*points)

    def boundingRect(self):
        size = self.theme.get('playhead_triangle_size', 10)
        return QRectF(-size / 2, 0, size, size)


class PlayheadLineItem(QGraphicsWidget):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setZValue(150)

    def paint(self, painter: QPainter, option, widget):
        scene = self.scene()
        height = scene.sceneRect().height() if scene else 0
        painter.setPen(QPen(QColor(self.theme['playhead_color']), 1, Qt.DashLine))
        painter.drawLine(0, 0, 0, height)

    def boundingRect(self):
        scene = self.scene()
        height = scene.sceneRect().height() if scene else 0
        return QRectF(0, 0, 1, height)


class EndLineItem(QGraphicsWidget):
    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.theme = theme
        self.setZValue(150)

    def paint(self, painter: QPainter, option, widget):
        scene = self.scene()
        height = scene.sceneRect().height() if scene else 0
        painter.setPen(QPen(QColor(self.theme['endline_color']), 2, Qt.SolidLine))
        painter.drawLine(0, 0, 0, height)

    def boundingRect(self):
        scene = self.scene()
        height = scene.sceneRect().height() if scene else 0
        return QRectF(-1, 0, 2, height)


class TrackHeaderItem(QGraphicsWidget):
    def __init__(self, track: TrackData, theme: dict, parent=None):
        super().__init__(parent)
        self.track = track
        self.theme = theme
        self.setZValue(10)

    def paint(self, painter: QPainter, option, widget):
        w = self.geometry().width()
        h = self.geometry().height()
        painter.fillRect(QRectF(0, 0, w, h), QBrush(QColor(self.theme['track_header_bg'])))
        font = QFont(
            self.theme.get('track_header_font_family', 'Arial'),
            self.theme.get('track_header_font_size', 10)
        )
        painter.setFont(font)
        painter.setPen(QPen(QColor(self.theme['track_header_fg'])))
        painter.drawText(
            QRectF(5, 0, w - 5, h),
            Qt.AlignVCenter | Qt.AlignLeft,
            self.track.name
        )

    def boundingRect(self):
        return QRectF(0, 0, self.geometry().width(), self.geometry().height())


class TrackLaneItem(QGraphicsWidget):
    def __init__(self, track: TrackData, theme: dict, parent=None):
        super().__init__(parent)
        self.track = track
        self.theme = theme
        self.setZValue(5)

    def paint(self, painter: QPainter, option, widget):
        w = self.geometry().width()
        h = self.geometry().height()
        rect = QRectF(0, 0, w, h)
        painter.fillRect(rect, QBrush(QColor(self.theme['track_lane_bg'])))
        pen = QPen(QColor(self.theme['track_lane_border']), 1)
        painter.setPen(pen)
        painter.drawRect(rect)

    def boundingRect(self):
        return QRectF(0, 0, self.geometry().width(), self.geometry().height())


class ClipItem(QGraphicsWidget):
    def __init__(self, clip: ClipData, track: TrackData, theme: dict, parent=None):
        super().__init__(parent)
        self.clip = clip
        self.track = track
        self.theme = theme
        self.setZValue(20)

    def paint(self, painter: QPainter, option, widget):
        w = self.geometry().width()
        h = self.geometry().height()
        rect = QRectF(0, 0, w, h)
        painter.fillRect(rect, QBrush(QColor(self.theme['clip_fill'])))
        painter.setPen(QPen(QColor(self.theme['clip_border']), 1))
        painter.drawRect(rect)
        font = QFont(
            self.theme.get('clip_font_family', 'Arial'),
            self.theme.get('clip_font_size', 8)
        )
        painter.setFont(font)
        painter.setPen(QPen(QColor(self.theme['clip_text'])))
        painter.drawText(
            rect.adjusted(2, 2, -2, -2),
            Qt.AlignLeft | Qt.AlignTop,
            str(self.clip.metadata.get('label', self.clip.track_name))
        )

    def boundingRect(self):
        return QRectF(0, 0, self.geometry().width(), self.geometry().height())
