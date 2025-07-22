# controller/timeline_controller.py

from PySide6.QtCore import QObject, QTimer, Signal, Slot
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from model.loaders import DataLoader
from model.data import TrackData, ClipData  # 必要なモデルクラスをインポート

class TimelineController(QObject):
    # シグナル定義
    data_loaded   = Signal(list)   # List[TrackData]
    frame_changed = Signal(int)    # 現在フレーム番号
    zoom_changed  = Signal(float)  # ズーム率

    def __init__(self, graphics_view: QGraphicsView):
        super().__init__()
        self.graphics_view = graphics_view
        # シーンを作成してビューにセット
        self.scene = QGraphicsScene(self)
        self.graphics_view.setScene(self.scene)

        # 再生タイマー（例: 24fps）
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._playback_step)

        # 状態
        self.tracks = []         # type: list[TrackData]
        self.current_frame = 0   # type: int
        self.zoom_factor = 1.0   # type: float

    @Slot(str, object)
    def load_data(self, path: str, loader: DataLoader):
        """
        DataLoader を使ってファイルから読み込み、
        TrackData のリストを受け取ってシーンに配置。
        """
        # モデル読み込み
        self.tracks = loader.load(path)

        # 既存アイテムをクリア
        self.scene.clear()

        # TrackData／ClipData 側で QGraphicsItem 化できるメソッドを用意しておく想定
        for track in self.tracks:
            for clip in track.clips:         # ClipData のリスト
                item = clip.to_graphics_item()  # ClipData#to_graphics_item() -> QGraphicsItem
                self.scene.addItem(item)

        # View（TimelineWidget）へ通知
        self.data_loaded.emit(self.tracks)

    @Slot()
    def play(self):
        """再生開始"""
        if not self.timer.isActive():
            # 24fps 想定
            interval = int(1000 / 24)
            self.timer.start(interval)

    @Slot()
    def pause(self):
        """再生停止"""
        if self.timer.isActive():
            self.timer.stop()

    def _playback_step(self):
        """タイマーごとの再生更新"""
        self.current_frame += 1

        # 各 QGraphicsItem にフレーム情報を渡して再描画を促す想定
        for item in self.scene.items():
            if hasattr(item, "update_frame"):
                item.update_frame(self.current_frame)

        # View 更新シグナル
        self.frame_changed.emit(self.current_frame)

    @Slot(float)
    def set_zoom(self, zoom: float):
        """ズーム率を更新し、ビューに反映"""
        self.zoom_factor = zoom
        self.graphics_view.resetTransform()
        self.graphics_view.scale(self.zoom_factor, 1.0)
        self.zoom_changed.emit(self.zoom_factor)
