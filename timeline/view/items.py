from PySide6.QtWidgets import QGraphicsWidget
from PySide6.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PySide6.QtCore import Qt, QRectF, QPointF

from utils import frames_to_timecode
from model.data import ClipData

# view/items.py (抜粋)

from PySide6.QtWidgets import QGraphicsItem
from PySide6.QtGui import QPolygonF, QPainter, QPen, QColor
from PySide6.QtCore import QRectF, Qt, QPointF


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


class RulerItem(QGraphicsItem):
    def __init__(self, view_model, parent=None):
        super().__init__(parent)
        self.h_zoom = view_model.h_zoom
        self.theme = view_model.theme
        self.view_model = view_model
        self.setZValue(10)

    def boundingRect(self) -> QRectF:
        # 元コード: シーン幅 - LEFT_MARGIN :contentReference[oaicite:12]{index=12}
        width = self.scene().width() - self.theme["LEFT_MARGIN"]
        return QRectF(0, 0, width, self.theme["TOP_MARGIN"])

    def paint(self, painter: QPainter, option, widget):
        # 元コード: major/minor tick 描画と時間コード :contentReference[oaicite:13]{index=13}
        rect = self.boundingRect()
        painter.fillRect(rect, QColor(self.theme["ruler_bg"]))
        fps = self.view_model.fps
        # view = self.scene().views()[0]
        scale = self.theme["BASE_PIXELS_PER_DECI_SECOND"] * self.h_zoom
        x = 0
        while x < rect.width():
            time = x / scale
            frame = int(time / 10 * self.view_model.shrink_rate * fps)
            if int(time) % 10 == 0:
                painter.setPen(QPen(QColor(self.theme["ruler_tick_major"])))
                painter.drawLine(x, rect.bottom(), x, rect.bottom() - 15)
                from utils import frames_to_timecode
                painter.drawText(x + 2, rect.bottom() - 17, frames_to_timecode(frame, fps))
            else:
                painter.setPen(QPen(QColor(self.theme["ruler_tick_minor"])))
                painter.drawLine(x, rect.bottom(), x, rect.bottom() - 5)
            x += scale


# class PlayheadTriangleItem(QGraphicsItem):
#     def __init__(self, view, theme: dict, view_model, parent=None):
#         super().__init__(parent)
#         self.view = view      # タイムライン制御用コールバックを受け取る
#         self.theme = theme
#         self.view_model = view_model
#         self.setFlags(
#             QGraphicsItem.ItemIsMovable
#             | QGraphicsItem.ItemSendsScenePositionChanges
#             | QGraphicsItem.ItemIgnoresTransformations
#         )
#         self.setZValue(1000)
#         self.dragging = False

#     def boundingRect(self) -> QRectF:
#         return QRectF(-10, 0, 20, self.theme["TOP_MARGIN"])

#     def paint(self, painter: QPainter, option, widget=None):
#         triangle_h = 15
#         triangle_w = 15
#         painter.setPen(Qt.NoPen)
#         painter.setBrush(QColor(self.theme["playhead_color"]))
#         tri = QPolygonF([
#             QPointF(-triangle_w/2, self.theme["TOP_MARGIN"] - triangle_h),
#             QPointF( triangle_w/2, self.theme["TOP_MARGIN"] - triangle_h),
#             QPointF(0,               self.theme["TOP_MARGIN"])
#         ])
#         painter.drawPolygon(tri)

#     def itemChange(self, change, value):
#         if change == QGraphicsItem.ItemPositionChange:
#             new_pos = value
#             new_pos.setY(0)
#             # view = self.scene().views()[0]
#             new_x = self.view_model._limit_x_current_line_move(new_pos.x()) # _view_model内でend_lineがstart~end_line_limit内に含まれているか判定
#             new_pos.setX(new_x)
#             return new_pos
#         # if change == QGraphicsItem.ItemPositionChange:
#         #     new_pos = value
#         #     # Y軸固定
#         #     new_pos.setY(0)
#         #     return new_pos
#         return super().itemChange(change, value)

#     def mousePressEvent(self, event):
#         self.view.pressed_playhead.emit()
#         # self.dragging = True
#         super().mousePressEvent(event)  # 選択状態変更など
    
#     # def mouseMoveEvent(self, event):
#     #     super().mouseMoveEvent(event)
#         # self._update_playhead()         # ドラッグに合わせて更新 :contentReference[oaicite:0]{index=0}

#     def mouseReleaseEvent(self, event):
#         new_current_frame = self.view_model._pix_to_frame(self.pos().x())
#         self.view.released_playhead.emit(new_current_frame)
#         # self._update_playhead()         # ドラッグ終了後も最終位置で更新 :contentReference[oaicite:1]{index=1}
#         # self.dragging = False
#         super().mouseReleaseEvent(event)

#     # def _update_playhead(self):
#         # view = self.scene().views()[0]
#         # new_frame = (self.pos().x() - self.theme["LEFT_MARGIN"]) / (
#         #     self.theme["BASE_PIXELS_PER_FRAME"] * view.h_zoom
#         # )
#         # コントローラに渡して、内部 playhead_frame を更新
#         # self.controller.set_playhead_frame(new_frame)
#         #   :contentReference[oaicite:2]{index=2}


# class PlayheadLineItem(QGraphicsItem):
#     def __init__(self, view, theme: dict, view_model, parent=None):
#         super().__init__(parent)
#         self.view = view
#         self.theme = theme
#         self.view_model = view_model
#         self.setFlags(
#             QGraphicsItem.ItemIsMovable
#             | QGraphicsItem.ItemSendsScenePositionChanges
#             | QGraphicsItem.ItemIgnoresTransformations
#         )
#         self.setZValue(1000)
#         self.dragging = False

#     def boundingRect(self) -> QRectF:
#         height = self.scene().height()
#         return QRectF(-2, 0, 4, height)

#     def paint(self, painter: QPainter, option, widget=None):
#         painter.fillRect(
#             self.boundingRect(),
#             QColor(self.theme["playhead_color"])
#         )

#     def itemChange(self, change, value):
#         if change == QGraphicsItem.ItemPositionChange:
#             new_pos = value
#             new_pos.setY(0)
#             # view = self.scene().views()[0]
#             new_x = self.view_model._limit_x_current_line_move(new_pos.x()) # _view_model内でend_lineがstart~end_line_limit内に含まれているか判定
#             new_pos.setX(new_x)
#             return new_pos
#         # if change == QGraphicsItem.ItemPositionChange:
#         #     new_pos = value
#         #     # Y軸はヘッダー下に固定
#         #     new_pos.setY(self.theme["TOP_MARGIN"])
#         #     return new_pos
#         return super().itemChange(change, value)

#     def mousePressEvent(self, event):
#         # self.dragging = True
#         self.view.pressed_playhead.emit()
#         super().mousePressEvent(event)
    
#     # def mouseMoveEvent(self, event):
#     #     super().mouseMoveEvent(event)
#         # self._update_playhead()         # ドラッグ中にも更新 :contentReference[oaicite:3]{index=3}

#     def mouseReleaseEvent(self, event):
#         new_current_frame = self.view_model._pix_to_frame(self.pos().x())
#         self.view.released_playhead.emit(new_current_frame)
#         # self._update_playhead()         # ドラッグ終了後に更新 :contentReference[oaicite:4]{index=4}
#         # self.dragging = False
#         super().mouseReleaseEvent(event)

#     # def _update_playhead(self):
#     #     view = self.scene().views()[0]
#     #     new_frame = (self.pos().x() - self.theme["LEFT_MARGIN"]) / (
#     #         self.theme["BASE_PIXELS_PER_FRAME"] * view.h_zoom
#     #     )
#     #     self.controller.set_playhead_frame(new_frame)

class PlayheadItem(QGraphicsItem):
    def __init__(self, view, theme: dict, view_model, parent=None):
        super().__init__(parent)
        self.view = view
        self.theme = theme
        self.view_model = view_model

        self.setFlags(
            QGraphicsItem.ItemIsMovable
            | QGraphicsItem.ItemSendsScenePositionChanges
            | QGraphicsItem.ItemIgnoresTransformations
        )
        self.setZValue(1000)

    def boundingRect(self) -> QRectF:
        height = self.scene().height() if self.scene() else 1000
        return QRectF(-10, 0, 20, height)

    def paint(self, painter: QPainter, option, widget=None):
        playhead_color = QColor(self.theme["playhead_color"])

        # ▼ 三角形の描画
        triangle_h = 15
        triangle_w = 15
        tri = QPolygonF([
            QPointF(-triangle_w/2, self.theme["TOP_MARGIN"] - triangle_h),
            QPointF( triangle_w/2, self.theme["TOP_MARGIN"] - triangle_h),
            QPointF(0,              self.theme["TOP_MARGIN"])
        ])
        painter.setPen(Qt.NoPen)
        painter.setBrush(playhead_color)
        painter.drawPolygon(tri)

        # │ 縦線の描画（ヘッダーの下から末尾まで）
        line_top = self.theme["TOP_MARGIN"]
        line_height = self.boundingRect().height() - line_top
        painter.fillRect(QRectF(-2, line_top, 4, line_height), playhead_color)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange:
            new_pos = value
            new_pos.setY(0)  # Yは固定
            new_x = self.view_model._limit_x_current_line_move(new_pos.x())
            new_pos.setX(new_x)
            return new_pos
        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        self.view.pressed_playhead.emit()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        new_current_frame = self.view_model._pix_to_frame(self.pos().x())
        self.view.released_playhead.emit(new_current_frame)
        super().mouseReleaseEvent(event)

class EndLineItem(QGraphicsItem):
    def __init__(self, view, theme: dict, view_model, parent=None):
        super().__init__(parent)
        self.view = view
        self.view_model = view_model
        self.theme = theme
        self.new_x = 0
        self.setFlags(
            QGraphicsItem.ItemIsMovable
            | QGraphicsItem.ItemSendsScenePositionChanges
            | QGraphicsItem.ItemIgnoresTransformations
        )
        self.setZValue(900)
        self.dragging = False

    def boundingRect(self) -> QRectF:
        # 元コード: boundingRect (高さ - BOTTOM_MARGIN) を使用 :contentReference[oaicite:0]{index=0}
        height = self.scene().height() - self.theme["BOTTOM_MARGIN"]
        return QRectF(-2, 0, 4, height)

    def paint(self, painter: QPainter, option, widget = None):
        # 元コード: 塗りつぶしに end_line_color を使用 :contentReference[oaicite:1]{index=1}
        painter.fillRect(self.boundingRect(), QColor(self.theme["end_line_color"]))

    def itemChange(self, change, value):
        # 元コード: Y位置固定＆最小 end フレームチェック :contentReference[oaicite:2]{index=2}
        if change == QGraphicsItem.ItemPositionChange:
            new_pos = value
            new_pos.setY(self.theme["TOP_MARGIN"])
            # view = self.scene().views()[0]
            new_x = self.view_model._limit_x_end_line_move(new_pos.x()) # _view_model内でend_lineがstart~end_line_limit内に含まれているか判定
            new_pos.setX(new_x)
            self.new_x = new_x
            return new_pos
        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        # 元コード: ドラッグ開始フラグ :contentReference[oaicite:3]{index=3}
        self.view.pressed_endline.emit()
        # self.dragging = True
        super().mousePressEvent(event)

    # def mouseMoveEvent(self, event):
    #     # 元コード: ドラッグ中に updateEndFrame() 呼び出し :contentReference[oaicite:4]{index=4}
    #     super().mouseMoveEvent(event)
    #     self.updateEndFrame()

    def mouseReleaseEvent(self, event):
        # 元コード: ドラッグ終了後に最終 updateEndFrame() & フラグ解除 :contentReference[oaicite:5]{index=5}
        new_end_frame = self.view_model._pix_to_frame(self.new_x)
        self.view.released_endline.emit(new_end_frame)
        # self.updateEndFrame()
        # self.dragging = False
        super().mouseReleaseEvent(event)

    # def updateEndFrame(self):
    #     # 元コード: シーン位置から endFrame を計算してコントローラに伝達 :contentReference[oaicite:6]{index=6}
    #     view = self.scene().views()[0]
    #     new_frame = (self.pos().x() - self.theme["LEFT_MARGIN"]) / (
    #         self.theme["BASE_PIXELS_PER_FRAME"] * view.h_zoom
    #     )
    #     self.controller.set_end_frame(new_frame)





class TimeLabelItem(QGraphicsItem):
    def __init__(self, playhead_frame=0, theme: dict = None, parent=None):
        super().__init__(parent)
        self.playhead_frame = playhead_frame
        self.theme = theme
        self.setZValue(20)

    def boundingRect(self) -> QRectF:
        # 元コード: LEFT_MARGIN × TOP_MARGIN :contentReference[oaicite:14]{index=14}
        return QRectF(0, 0, self.theme["LEFT_MARGIN"], self.theme["TOP_MARGIN"])

    def paint(self, painter: QPainter, option, widget):
        # 元コード: 背景＋中心揃えテキスト :contentReference[oaicite:15]{index=15}
        rect = self.boundingRect()
        painter.fillRect(rect, QColor(self.theme["timeLabel_bg"]))
        painter.setPen(QColor(self.theme["timeLabel_text"]))
        painter.setFont(QFont("Sans", 10))
        from utils import frames_to_timecode
        text = frames_to_timecode(self.playhead_frame)
        painter.drawText(rect, Qt.AlignCenter, text)

    def updateTime(self, frame: float):
        # 元コード: フレーム更新 → 再描画 :contentReference[oaicite:16]{index=16}
        self.playhead_frame = int(round(frame))
        self.update()


class TrackHeaderItem(QGraphicsItem):
    def __init__(self, track_data, view_model, parent=None):
        super().__init__(parent)
        self.track_data = track_data
        self.theme = view_model.theme
        self.v_zoom = view_model.v_zoom
        self.setZValue(5)

    def boundingRect(self) -> QRectF:
        # 元コード: LEFT_MARGIN × (height × v_zoom) :contentReference[oaicite:17]{index=17}
        # v_zoom = self.scene().views()[0].v_zoom
        return QRectF(
            0, 0,
            self.theme["LEFT_MARGIN"],
            self.track_data.height * self.v_zoom
        )

    def paint(self, painter: QPainter, option, widget):
        # 元コード: 背景・テキスト・下線描画 :contentReference[oaicite:18]{index=18} and :contentReference[oaicite:19]{index=19}
        rect = self.boundingRect()
        painter.fillRect(rect, QColor(self.theme["track_header_bg"]))
        painter.setPen(QColor(self.theme["track_header_text"]))
        painter.setFont(QFont("Sans", 10))
        painter.drawText(rect.adjusted(5, 0, -5, 0),
                         Qt.AlignVCenter | Qt.AlignLeft,
                         self.track_data.name)
        painter.setPen(QPen(QColor(
            self.theme.get("track_header_border", self.theme["track_lane_border"])
        )))
        painter.drawLine(rect.bottomLeft(), rect.bottomRight())


class TrackLaneItem(QGraphicsItem):
    def __init__(self, track_data, theme: dict = None, parent=None):
        super().__init__(parent)
        self.track_data = track_data
        self.theme = theme
        self.rect = QRectF()
        self.setZValue(0)

    def boundingRect(self) -> QRectF:
        return self.rect  # :contentReference[oaicite:20]{index=20}

    def setGeometry(self, x, y, width, height):
        self.rect = QRectF(x, y, width, height)
        self.update()

    def paint(self, painter: QPainter, option, widget):
        # 元コード: 条件に応じた背景色＆枠線 :contentReference[oaicite:21]{index=21}
        bg = (
            QColor(self.theme["track_lane_bg1"])
            if self.track_data.name.endswith("1")
            else QColor(self.theme["track_lane_bg2"])
        )
        painter.fillRect(self.rect, bg)
        painter.setPen(QPen(QColor(self.theme["track_lane_border"])))
        painter.drawRect(self.rect)



class ClipItem(QGraphicsItem):
    SNAP_TOLERANCE = 1

    def __init__(self, clip_data: ClipData, track_data, view_model, is_moveable:bool, parent=None):
        super().__init__(parent)
        self.clip_data = clip_data
        self.track_data = track_data
        self.theme = view_model.theme
        self.h_zoom = view_model.h_zoom
        self.rect = QRectF()
        if is_moveable:
            self.setFlags(
                QGraphicsItem.ItemIsSelectable |
                QGraphicsItem.ItemIsMovable
            )
        self._fixed_y = 0

    def boundingRect(self) -> QRectF:
        return self.rect

    def setGeometry(self, x, y, width, height):
        self.rect = QRectF(0, 0, width, height)
        self.setPos(x, y)
        self._fixed_y = y
        self.update()

    def paint(self, painter: QPainter, option, widget):
        fill = (
            QColor(self.theme["clip_fill_selected"])
            if self.isSelected()
            else QColor(self.theme["clip_fill"])
        )
        painter.fillRect(self.rect, fill)
        painter.setPen(QPen(QColor(self.theme["clip_border"])))
        painter.drawRect(self.rect)
        painter.setPen(QColor(self.theme["track_header_text"]))
        painter.setFont(QFont("Sans", 8))
        margin = 2
        # 時間コードの表示
        from utils import frames_to_timecode
        start_tc = frames_to_timecode(self.clip_data.start_frame)  # :contentReference[oaicite:7]{index=7}
        end_tc = frames_to_timecode(
            self.clip_data.start_frame + self.clip_data.duration_frames
        )
        painter.drawText(
            self.rect.adjusted(margin, margin, 0, 0),
            Qt.AlignLeft | Qt.AlignTop,
            start_tc,
        )
        painter.drawText(
            self.rect.adjusted(0, margin, -margin, 0),
            Qt.AlignRight | Qt.AlignTop,
            end_tc,
        )
        painter.drawText(self.rect, Qt.AlignCenter, getattr(self.clip_data, "title", ""))

    def itemChange(self, change, value):
        # 元コード: Y固定 :contentReference[oaicite:8]{index=8}
        if change == QGraphicsItem.ItemPositionChange:
            new_pos = value
            new_pos.setY(self._fixed_y)
            return new_pos
        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        # 元コード: Y位置の記憶 :contentReference[oaicite:9]{index=9}
        self._fixed_y = self.pos().y()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        # 元コード: スナップ候補生成・重なりチェック・配置更新 :contentReference[oaicite:10]{index=10} :contentReference[oaicite:11]{index=11}
        view = self.scene().views()[0]
        original = round(
            (self.pos().x() - self.theme["LEFT_MARGIN"])
            / (self.theme["BASE_PIXELS_PER_FRAME"] * self.h_zoom)
        )
        # 候補リスト構築
        options = [original]
        for other in self.track_data.clips:
            if other is self.clip_data:
                continue
            end = other.start_frame + other.duration_frames
            if abs(original - end) <= self.SNAP_TOLERANCE:
                options.append(end)
            if abs(original + self.clip_data.duration_frames - other.start_frame) <= self.SNAP_TOLERANCE:
                options.append(other.start_frame - self.clip_data.duration_frames)
        # 最適候補選択
        candidate = min(options, key=lambda c: abs(c - original))
        # 重なりチェック
        for other in self.track_data.clips:
            if other is self.clip_data:
                continue
            if (candidate < other.start_frame + other.duration_frames
                    and candidate + self.clip_data.duration_frames > other.start_frame):
                candidate = original
                break
        # 更新＆移動
        self.clip_data.start_frame = max(0, candidate)
        new_x = (
            self.theme["LEFT_MARGIN"]
            + self.clip_data.start_frame
            * self.theme["BASE_PIXELS_PER_FRAME"]
            * self.h_zoom
        )
        self.setPos(new_x, self._fixed_y)
        super().mouseReleaseEvent(event)

