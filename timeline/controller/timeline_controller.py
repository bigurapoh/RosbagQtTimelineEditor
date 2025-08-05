# controller/timeline_controller.py
from typing import List
from PySide6.QtCore import QObject, QTimer, Signal, Slot
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene
from model.loaders import DataLoader, CsvLoader, RosbagLoader
from model.data import TrackMetaData, ClipData, RecordData, RecordMngModel, TimelineModel  # 必要なモデルクラスをインポート
from view.items import ClipItem  # 追加：ClipItem を直接利用

import time

class TimelineController(QObject):
    # シグナル定義
    data_loaded   = Signal()   # List[TrackData]
    frame_changed = Signal()    # 現在フレーム番号
    end_frame_changed = Signal()
    # zoom_changed  = Signal(float)  # ズーム率

    def __init__(self, record_mng_model:RecordMngModel, timeline_model:TimelineModel, view):
        super().__init__()
        # self.theme = theme
        # self.graphics_view = graphics_view
        # シーンを作成してビューにセット
        # self.scene = QGraphicsScene(self)
        # self.graphics_view.setScene(self.scene)
        self.record_mng_model = record_mng_model
        self.timeline_model = timeline_model


        # 再生タイマー（例: 24fps）
        self.loader_fps = DataLoader.loader_fps
        self.timer_fps = self.loader_fps # :future: timerのfpsをずらす機能 => 0.5倍再生など
        self.timer = QTimer(view)
        self.interval = int(1000/self.timer_fps) # :Check: # 24fps 想定
        self.timer.timeout.connect(self._playback_step)

        self.timeline_model.over_end_line.connect(self.pause)

        # 状態
        self.tracks = []         # type: list[TrackMetaData]
        self.current_frame = 0   # type: int
        self.zoom_factor = 1.0   # type: float
        # クリップ配置用の内部状態
        self.items = []
        # # 各トラックの縦幅（ピクセル）
        # self.default_track_height = self.theme["DEFAULT_TRACK_HEIGHT"]

    #!--- 4. --- Done ---self.controllerにadd_recordを実装する(loaderとの兼ね合い,loaderいらないかも. rosbagデータのloaderも実装できるから実はほしいかも)--- Done
    def add_record(self, path:str=None, record_data:RecordData=None):
        if path is not None:
            # if not path: return
            ext = path.rsplit(".",1)[-1].lower()
            loader:DataLoader = {"csv":CsvLoader,"bag":RosbagLoader}.get(ext)()
            loaded_record_data = loader.load(path)
            all_records_end_frame = self.record_mng_model.add_record(loaded_record_data)
            # self.controller.load_data(path, loader)
            self.timeline_model.set_end_line_frame(all_records_end_frame) # 新しいレコードを読み込む度にendlineの位置を初期化する # :future: startlineの初期化
            self.end_frame_changed.emit()
            self.data_loaded.emit()
            return

        if record_data is not None:
            all_records_end_frame = self.record_mng_model.add_record(record_data)
            # self.controller.load_data(path, loader)
            self.timeline_model.set_end_line_frame(all_records_end_frame) # 新しいレコードを読み込む度にendlineの位置を初期化する # :future: startlineの初期化
            self.end_frame_changed.emit()
            self.data_loaded.emit()
            return
    # def add_record(self, record_data:RecordData):
    #     self.record_mng_model.add_record(record_data)

    # @Slot(str, object)
    # def load_data(self, path: str, loader: DataLoader):
    #     """
    #     DataLoader を使ってファイルから読み込み、
    #     load_tracks() へ委譲。
    #     """
    #     self.tracks = loader.load(path)
    #     self.load_tracks(self.tracks)

    # @Slot(list)
    # def load_tracks(self, tracks: List[TrackMetaData]):
    #     """
    #     TrackData のリストを受け取ってシーンに配置。
    #     """
    #     self.scene.clear()
    #     for ti, track in enumerate(tracks):
    #         # Y オフセットはトラック順に定義（定数 or theme から取得）
    #         print(ti, self.theme["DEFAULT_TRACK_HEIGHT"])
    #         y_offset = ti * self.theme["DEFAULT_TRACK_HEIGHT"]
    #         for clip in track.clips:
    #             # ClipItem を直接生成
    #             item = ClipItem(clip, track, self.theme)
    #             # ジオメトリを計算（水平ズーム・基本ピクセル/フレームを反映）
    #             x = self.theme["BASE_PIXELS_PER_FRAME"] * clip.start_frame * self.zoom_factor
    #             w = self.theme["BASE_PIXELS_PER_FRAME"] * clip.duration_frames * self.zoom_factor
    #             item.setGeometry(x, y_offset, w, track.height)
    #             self.scene.addItem(item)
    #     # View 側に再描画完了を通知
    #     self.data_loaded.emit(tracks)
    
    #!--- 1.
    @Slot()
    def play(self):
        """再生開始"""
        if self.record_mng_model.has_record:
            if not self.timer.isActive():
                self.timer.start(self.interval)

    @Slot()
    def pause(self):
        """再生停止"""
        if self.timer.isActive():
            self.timer.stop()

    @Slot()
    def go_start_frame(self):
        """再生を停止してStart_Lineにフレームを移動"""
        self.pause()
        self.timeline_model.go_start_frame()
        self.frame_changed.emit()

    @Slot()
    def go_end_frame(self):
        """再生を停止してEnd_Lineにフレームを移動"""
        self.pause()
        self.timeline_model.go_end_frame()
        self.frame_changed.emit()

    @Slot(int)
    def release_end_line(self, to_frame):
        """(一応)再生を停止してUIによって与えられたend_frameをmodelにセット"""
        self.pause()
        self.timeline_model.set_end_line_frame(to_frame)
        self.end_frame_changed.emit()
        self.go_start_frame()

    @Slot(int)
    def release_current_line(self, to_frame):
        """(一応)再生を停止してUIによって与えられたcurrent_frameをmodelにセット"""
        self.pause()
        self.timeline_model.set_current_line_frame(to_frame)

    # :Future: move_start_line

    # :Future: Loop再生機能対応

    #!--- 5.update_sceneはmodelから発火する?, そもそも変更がない部分のviewは小さく更新したい気持ちもあるが、最初の実装としては一旦まとめて行う
    def _playback_step(self):
        """タイマーごとの再生更新"""
        f_curr = self.timeline_model.next()
        self.record_mng_model.now(f_curr)
        self.frame_changed.emit()
        # self.current_frame += 1

        # # 各 QGraphicsItem にフレーム情報を渡して再描画を促す想定
        # for item in self.scene.items():
        #     if hasattr(item, "update_frame"):
        #         item.update_frame(self.current_frame)

        # # View 更新シグナル
        # self.frame_changed.emit(self.current_frame)

    #!--- 6.これはviewに移すべきロジック? --- Done
    # @Slot(float)
    # def set_zoom(self, zoom: float):
    #     """ズーム率を更新し、ビューに反映"""
    #     self.zoom_factor = zoom
    #     self.graphics_view.resetTransform()
    #     self.graphics_view.scale(self.zoom_factor, 1.0)
    #     self.zoom_changed.emit(self.zoom_factor)
    
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
