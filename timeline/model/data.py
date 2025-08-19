# model/data.py

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional

from PySide6.QtCore import QObject, Signal

@dataclass
class ClipData:
    """
    タイトル付きクリップデータ。duration_frames→end_frameを自動計算。
    """
    title: str = ""
    start_frame: int = 0
    duration_frames: int = 1
    track_id: Optional[int] = None

    @property
    def end_frame(self) -> int:
        # start_frame から duration_frames で終端を計算
        return self.start_frame + self.duration_frames - 1

    def duration(self) -> int:
        return self.duration_frames


@dataclass
class TrackMetaData:
    """
    名前・高さ付きトラックデータ。スクリプトとローダー両対応。
    """
    id: Optional[int] = None # 特に使用されないので原則未定義で良い
    name: str = ""
    ui_height: float = 30.0 # viewのgeometry設定に利用するheight.
    is_movable: bool = False #!-- Trueは未実装 --! # clipの移動を有効にするフラグ
    clips: List[ClipData] = field(default_factory=list)

    @property
    def height(self):
        return self.ui_height

    def total_duration(self) -> int:
        return sum(c.duration() for c in self.clips)
    
    # スクリプト用のメソッド(Mock用)
    def add_clip(self, clip: ClipData) -> None:
        # track_id を設定のうえ追加
        clip.track_id = self.id
        self.clips.append(clip)




# TrackMetaDataのリストをpropertyとして持つクラス群の定義
# bagファイルなど一つのファイルに対応するインスタンス
class RecordData(ABC):
    """
    metadataとrecordsを持つデータ.
    現在のframeを受け取ってそれぞれの動作をnowメソッドにより行う.
    """
    @abstractmethod
    def now(self, frame: int):...
    @property
    @abstractmethod
    def metadatas(self) -> List[TrackMetaData]:...
    @property
    @abstractmethod
    def end_frame(self):...
    @property
    @abstractmethod
    def fps(self): ...


class RosbagData(RecordData):
    def __init__(self, track_metadatas: List[TrackMetaData], track_records, fps:int):
        """
        track_metadatas:    List[TrackMetaData] => 特に .name="topic名", .clips[0].start_frame, .clips[0].end_frameが重要(RosbagDataではclipsは長さ1で固定) 
        track_records:      {"tpc1": (record1, topic_type), "tpc2": (record2, topic_type2), ...}という形式のdictがloaderによって用意される.
                            また各recordは{frame0: msg0, frame1: msg1, ...}という形式で保存されており、topic_typeは実際の型クラスがそのまま格納されている
        fps:                track_recordsにおけるframeの計算に用いたfpsを登録
        """
        import rospy

        self.track_metadatas = track_metadatas
        self.track_records = track_records 
        self._fps = fps
        
        # 最大値計算処理
        self.max_end_frame = 0
        for t in track_metadatas:
            for c in t.clips:
                self.max_end_frame = max(self.max_end_frame, c.end_frame)

        # {topic名: Publisher}形式の辞書を用意する
        rospy.init_node("rosbag-palyer", anonymous=False)
        self.pub_dict = {}
        for t in track_metadatas:
            topic_type = self.track_records[t.name][1]
            self.pub_dict[t.name] = rospy.Publisher(t.name, topic_type, queue_size=1)
    
    @property
    def metadatas(self):
        return self.track_metadatas

    @property
    def end_frame(self):
        return self.max_end_frame
    
    @property
    def fps(self):
        return self._fps
    
    def now(self, frame):
        # recordからframe_idxによりその値を返す
        for t in self.track_metadatas:
            record_dict = self.track_records[t.name][0]
            now_msg = record_dict[frame]
            
            # now_recordが存在すればpublish
            if now_msg is not None:
                self.pub_dict[t.name].publish(now_msg)

class MockData(RecordData):
    def __init__(self, track_metadatas: List[TrackMetaData], fps:int=24):
        """
        Mock用のRecord_Data.
        初期化時のtrack_metadatasから更新する処理を実装していないため,
        add_clipsなどをすると不具合が生じる可能性あり.
        """
        self.track_metadatas = track_metadatas
        self._fps = fps
        
        # 最大値計算処理
        self.max_end_frame = 0
        for t in track_metadatas:
            for c in t.clips:
                self.max_end_frame = max(self.max_end_frame, c.end_frame)
    
    @property
    def metadatas(self):
        return self.track_metadatas

    @property
    def end_frame(self):
        return self.max_end_frame
    
    @property
    def fps(self):
        return self._fps
    
    def now(self, frame):
        pass

class RecordMngModel():
    """
    登録された全てのRecordDataを一括で管理するMng.
    nowメソッドにより全てのRecordDataのnowを発火する
    """
    def __init__(self):
        self.record_data_list :List[RecordData] = []
        self.all_records_end_frame = 0
        self.all_track_metadatas :List[TrackMetaData] = []
    
    @property
    def has_record(self):
        return bool(self.record_data_list)

    def add_record(self, record_data:RecordData):
        self.record_data_list.append(record_data)
        self.all_track_metadatas.extend(record_data.metadatas)

        # 最大frameの計算
        self.all_records_end_frame = max(self.all_records_end_frame, record_data.end_frame)

        return self.all_records_end_frame

    def now(self, frame):
        for record_data in self.record_data_list:
            record_data.now(frame)
        #!--- 11.実際のtimecodeの返却とframeによるtimecodeの返却どちらにも対応できるようにする(future)

class TimelineModel(QObject):
    over_end_line = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.start_frame = 0
        self.current_frame = 0
        self.end_frame = 0

    def go_start_frame(self):
        self.current_frame = self.start_frame

    def go_end_frame(self):
        self.current_frame = self.end_frame

    def set_end_line_frame(self, frame):
        self.end_frame = frame

    def set_current_line_frame(self, frame):
        self.current_frame = frame

    def next(self):
        self.current_frame += 1
        if self.current_frame >= self.end_frame:
            self.over_end_line.emit()