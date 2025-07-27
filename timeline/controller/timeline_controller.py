# controller/timeline_controller.py
from typing import List
from PySide6.QtCore import QObject, QTimer, Signal, Slot
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from model.loaders import DataLoader
from model.data import TrackData, ClipData  # 必要なモデルクラスをインポート
from view.items import ClipItem  # 追加：ClipItem を直接利用

class TimelineController(QObject):
    # シグナル定義
    data_loaded   = Signal(list)   # List[TrackData]
    frame_changed = Signal(int)    # 現在フレーム番号
    zoom_changed  = Signal(float)  # ズーム率

    def __init__(self, graphics_view: QGraphicsView, theme):
        super().__init__()
        self.theme = theme
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
        # クリップ配置用の内部状態
        self.items = []
        # # 各トラックの縦幅（ピクセル）
        # self.default_track_height = self.theme["DEFAULT_TRACK_HEIGHT"]

    #!--- 4.self.controllerにadd_trackを実装する(loaderとの兼ね合い,loaderいらないかも. rosbagデータのloaderも実装できるから実はほしいかも)
    @Slot(str, object)
    def load_data(self, path: str, loader: DataLoader):
        """
        DataLoader を使ってファイルから読み込み、
        load_tracks() へ委譲。
        """
        self.tracks = loader.load(path)
        self.load_tracks(self.tracks)

    @Slot(list)
    def load_tracks(self, tracks: List[TrackData]):
        """
        TrackData のリストを受け取ってシーンに配置。
        """
        self.scene.clear()
        for ti, track in enumerate(tracks):
            # Y オフセットはトラック順に定義（定数 or theme から取得）
            print(ti, self.theme["DEFAULT_TRACK_HEIGHT"])
            y_offset = ti * self.theme["DEFAULT_TRACK_HEIGHT"]
            for clip in track.clips:
                # ClipItem を直接生成
                item = ClipItem(clip, track, self.theme)
                # ジオメトリを計算（水平ズーム・基本ピクセル/フレームを反映）
                x = self.theme["BASE_PIXELS_PER_FRAME"] * clip.start_frame * self.zoom_factor
                w = self.theme["BASE_PIXELS_PER_FRAME"] * clip.duration_frames * self.zoom_factor
                item.setGeometry(x, y_offset, w, track.height)
                self.scene.addItem(item)
        # View 側に再描画完了を通知
        self.data_loaded.emit(tracks)
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

    #!--- 5.update_sceneはmodelから発火する?, そもそも変更がない部分のviewは小さく更新したい気持ちもあるが、最初の実装としては一旦まとめて行う
    def _playback_step(self):
        """タイマーごとの再生更新"""
        self.current_frame += 1

        # 各 QGraphicsItem にフレーム情報を渡して再描画を促す想定
        for item in self.scene.items():
            if hasattr(item, "update_frame"):
                item.update_frame(self.current_frame)

        # View 更新シグナル
        self.frame_changed.emit(self.current_frame)

    #!--- 6.これはviewに移すべきロジック?
    @Slot(float)
    def set_zoom(self, zoom: float):
        """ズーム率を更新し、ビューに反映"""
        self.zoom_factor = zoom
        self.graphics_view.resetTransform()
        self.graphics_view.scale(self.zoom_factor, 1.0)
        self.zoom_changed.emit(self.zoom_factor)
    
    # @Slot(list)
    # def load_tracks(self, tracks: List[TrackData]):
    #     """
    #     スクリプトから渡されたトラック一覧をシーンに配置。
    #     load_data(file, loader) とほぼ同様の動作を行う。
    #     """
    #     self.tracks = tracks
    #     self.scene.clear()
    #     self.items.clear()

    #     for idx, track in enumerate(tracks):
    #         y = idx * self.track_height
    #         for clip in track.clips:
    #             # track_id が未設定なら設定
    #             clip.track_id = track.id
    #             item = create_clip_item(clip, y_offset=y, scale=self.zoom_factor)
    #             self.scene.addItem(item)
    #             self.items.append(item)

    #     self.data_loaded.emit(self.tracks)
