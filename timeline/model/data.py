# model/data.py

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any

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
    id: Optional[int] = None
    name: str = ""
    height: float = 30.0 # ToDO: heightがdataには不必要なプログラムにする?
    clips: List[ClipData] = field(default_factory=list)

    def total_duration(self) -> int:
        return sum(c.duration() for c in self.clips)
    
    # スクリプト用のメソッド(Mock用)
    def add_clip(self, clip: ClipData) -> None:
        # track_id を設定のうえ追加
        clip.track_id = self.id
        self.clips.append(clip)




#!--- 8.MockDataとRosbagDataの追加.
class RecordData(ABC):
    """
    metadataとrecordsを持つデータ.
    現在のframeを受け取ってそれぞれの動作をnowメソッドにより行う.
    """
    @abstractmethod
    def now(self, frame: int):...
    @abstractmethod
    @property
    def metadatas(self) -> List[TrackMetaData]:...


class RosbagData(RecordData):
    def __init__(self, track_metadatas: List[TrackMetaData], track_records):
        self.track_metadatas = track_metadatas
        self.track_records = track_records # loaderによってframeをkeyにした辞書のリストとして作成される

        # ToDO: publisherの用意
    
    @property
    def metadatas(self):
        return self.track_metadatas
    
    def now(self, frame):
        # ToDo: recordsからframe_idxによりその値を返す
        for track_record in self.track_records:
            now_record = track_record # frameがkeyとして存在しなければNone, 存在すればvalueが返却される 

        # now_recordが存在すればpublish
        pass


class MockData(RecordData):
    def __init__(self, track_metadatas: List[TrackMetaData]):
        self.track_metadatas = track_metadatas
    
    @property
    def metadatas(self):
        return self.track_metadatas
    
    def now(frame):
        pass


#!--- 9.TimelineMngModelの実装. どのようなメソッドを実装すべきかをcontrollerとの兼ね合いで考える
class TimlineMngModel():
    """
    登録された全てのRecordDataを一括で管理するMng.
    nowメソッドにより全てのRecordDataのnowを発火する
    """
    def __init__(self):
        pass

    def add_records():
        pass