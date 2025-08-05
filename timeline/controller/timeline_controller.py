# controller/timeline_controller.py
from PySide6.QtCore import QObject, QTimer, Signal, Slot
from model.loaders import DataLoader, CsvLoader, RosbagLoader
from model.data import TrackMetaData, RecordData, RecordMngModel, TimelineModel

class TimelineController(QObject):
    # シグナル定義
    data_loaded   = Signal()   # List[TrackData]
    frame_changed = Signal()    # 現在フレーム番号
    end_frame_changed = Signal()

    def __init__(self, record_mng_model:RecordMngModel, timeline_model:TimelineModel, view):
        super().__init__()
        self.record_mng_model = record_mng_model
        self.timeline_model = timeline_model


        # 実際のタイマー再生用のfpsと、計算に活用するモデル用のfpsを作成
        self.loader_fps = DataLoader.loader_fps # モデルすべてに統一された計算用fpsを用意
        self.timer_fps = self.loader_fps # :future: timerのfpsをずらす機能 => 0.5倍再生など
        
        # 再生タイマー
        self.timer = QTimer(view)
        self.interval = int(1000/self.timer_fps) # :Check: 24fps 想定
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

    
    # プログラムからcontroller経由でモデルにrecordを追加する
    def add_record(self, path:str=None, record_data:RecordData=None):
        if path is not None:
            ext = path.rsplit(".",1)[-1].lower()
            loader:DataLoader = {"csv":CsvLoader,"bag":RosbagLoader}.get(ext)()
            loaded_record_data = loader.load(path)
            all_records_end_frame = self.record_mng_model.add_record(loaded_record_data)
            self.timeline_model.set_end_line_frame(all_records_end_frame) # 新しいレコードを読み込む度にendlineの位置を初期化する # :future: startlineの初期化
            self.end_frame_changed.emit()
            self.data_loaded.emit()
            return

        if record_data is not None:
            all_records_end_frame = self.record_mng_model.add_record(record_data)
            self.timeline_model.set_end_line_frame(all_records_end_frame) # 新しいレコードを読み込む度にendlineの位置を初期化する # :future: startlineの初期化
            self.end_frame_changed.emit()
            self.data_loaded.emit()
            return
    

    
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

    #!--- そもそも変更がない部分のviewは小さく更新したい気持ちもあるが、最初の実装としては一旦まとめて行う
    def _playback_step(self):
        """タイマーごとの再生更新"""
        f_curr = self.timeline_model.next()
        self.record_mng_model.now(f_curr)
        self.frame_changed.emit()
