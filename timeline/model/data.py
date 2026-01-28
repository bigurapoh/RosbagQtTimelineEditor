# model/data.py

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

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
    def shared_start_time(self):...
    @property
    @abstractmethod
    def start_frame(self):...
    @property
    @abstractmethod
    def end_frame(self):...
    @property
    @abstractmethod
    def fps(self): ...

def _calc_min_start_frame(track_metadatas:List[TrackMetaData]):
    start_frames = []
    for t in track_metadatas:
        for c in t.clips:
            start_frames.append(c.start_frame)

    return min(start_frames)

def _calc_max_end_frame(track_metadatas:List[TrackMetaData]):
    # 最大値計算処理
    max_end_frame = 0
    for t in track_metadatas:
        for c in t.clips:
            max_end_frame = max(max_end_frame, c.end_frame)
    
    return max_end_frame


# loaderによって事前生成された、時系列で変化するグラフを描画するクラス(時間軸がないグラフの描画を扱う)
class AnimeGraphData(RecordData):
    def __init__(self, track_metadatas:List[TrackMetaData], track_records, fps:int, concat_axis=0, start_time=None):
        """
        track_metadatas:    TrackMetaDataクラスのリスト.  name属性はカラム名
        track_records:      各track_nameに対するgraph_dictが保存された辞書型データ. なお各trackは連結されて描画される(基本単独trackのデータを想定しているが、複数存在する場合には連結する)
        (graph_dict):       frameをkey, その時刻のグラフのcv2形式(numpy形式)画像をvalueとしたdict
        fps:                track_recordsにおけるframeの計算に用いたfpsを登録
        concat_axis:        カラム(track)の連結方向を指定する. 0が横、1が縦
        """
        self.track_metadatas = track_metadatas
        self.track_records:dict = track_records
        self._fps = fps
        self._start_time = start_time

        if len(self.track_metadatas) > 1:
            self.concat_axis = concat_axis
        elif len(self.track_metadatas) == 1:
            self.concat_axis = None
        else:
            raise ValueError("track_metadatas is empty.")
        
        self.min_start_frame = _calc_min_start_frame(self.track_metadatas)
        self.max_end_frame = _calc_max_end_frame(self.track_metadatas)

    def now(self, frame):
        if self.concat_axis:
            pass # To Do: concat機能
        else:
            if frame in self.track_records["distances"].keys():
                img_bgr = self.track_records["distances"][frame].copy() # 表示する画像をコピー
                cv2.imshow('Graph', img_bgr)
    
    @property
    def metadatas(self):
        return self.track_metadatas

    @property
    def shared_start_time(self):
        return self._start_time

    @property
    def start_frame(self):
        return self.min_start_frame

    @property
    def end_frame(self):
        return self.max_end_frame
    
    @property
    def fps(self):
        return self._fps

# loaderによって事前生成された横軸が時間のグラフに対して、現在のフレームをを描画するクラス
class StaticTimeGraphData(RecordData):
    def __init__(self):
        """
        複数のトラックをタブで選択できるようにする
        """
        pass

    def now(self, frame):
        pass
    
    @property
    def metadatas(self):
        return self.track_metadatas

    @property
    def end_frame(self):
        return self.max_end_frame

# rosbagのpublishを行うクラス
class RosbagData(RecordData):
    def __init__(self, track_metadatas: List[TrackMetaData], track_records, fps:int, start_time=None):
        """
        track_metadatas:    TrackMetaDataクラスのリスト.  name属性はトピック名. (基本各trackにclipは単独で存在するため、len(clips)=1) 
        track_records:      keyがトピック名(track_name)、valueがrecord_dict及びros_publish用のトピックの型クラスのタプルという形式の辞書型データ.
        (record_dict):      frameをkey, その時刻のmsgをvalueとしたdict
        fps:                track_recordsにおけるframeの計算に用いたfpsを登録
        """
        import rospy

        self.track_metadatas = track_metadatas
        self.track_records = track_records 
        self._fps = fps
        self._start_time = start_time
        
        self.min_start_frame = _calc_min_start_frame(self.track_metadatas)
        self.max_end_frame = _calc_max_end_frame(self.track_metadatas)

        # {topic名: Publisher}形式の辞書を用意する
        rospy.init_node("rosbag_palyer", anonymous=False)
        self.pub_dict = {}
        for t in track_metadatas:
            topic_type = self.track_records[t.name][1]
            self.pub_dict[t.name] = rospy.Publisher(t.name, topic_type, queue_size=1)
    
    @property
    def metadatas(self):
        return self.track_metadatas

    @property
    def start_frame(self):
        return self.min_start_frame

    @property
    def end_frame(self):
        return self.max_end_frame
    
    @property
    def fps(self):
        return self._fps

    @property
    def shared_start_time(self):
        return self._start_time
    
    def now(self, frame):
        # recordからframe_idxによりその値を返す
        for t in self.track_metadatas:
            record_dict = self.track_records[t.name][0]
            now_msg = record_dict.get(frame)
            
            # now_recordが存在すればpublish
            if now_msg is not None:
                self.pub_dict[t.name].publish(now_msg)


class RosbagWithBlurData(RosbagData):
    camera_topic_name = "/camera/color/image_raw"
    def __init__(self, track_metadatas, track_records, fps, start_time=None):
        super().__init__(track_metadatas, track_records, fps, start_time)

        from cv_bridge import CvBridge
        bridge = CvBridge()
        
        in_camera_topic = False
        for t in self.track_metadatas:
            if t.name == self.camera_topic_name:
                record_dict:dict = self.track_records[t.name][0]
            
                # blur_dictの構築
                self.blur_dict = {}
                for frame_key, msg in record_dict.items():
                    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    blur = cv2.Laplacian(img, cv2.CV_64F).var()
                    self.blur_dict[frame_key] = blur
                in_camera_topic = True
                break
        
        if not in_camera_topic:
            raise ValueError(f"cameraトピックが含まれていません: {self.camera_topic_name}")
        self.prepare_static_graph()

    def prepare_static_graph(self):
        
        # グラフ描画とキャッシュ
        self.fig, self.ax = plt.subplots()
        x_vals = list(self.blur_dict.keys())
        y_vals = list(self.blur_dict.values())

        self.ax.plot(x_vals, y_vals, linestyle='-')
        self.ax.grid(True)

        self.fig.canvas.draw()
        self.img_rgb = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        self.img_rgb = self.img_rgb.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        self.img_bgr_base = cv2.cvtColor(self.img_rgb, cv2.COLOR_RGB2BGR)

        # Store transformation function: data coords → pixel coords
        self.trans_data_to_pixels = self.ax.transData.transform
        self.canvas_width, self.canvas_height = self.fig.canvas.get_width_height()

        plt.close(self.fig)  # グラフ表示は不要なので閉じる
    
    def now(self, frame):
        super().now(frame)

        # ベース画像をコピー
        img_bgr = self.img_bgr_base.copy()

        # matplotlib の座標変換を使って frame → pixel の x 座標に変換
        pixel_coord = self.trans_data_to_pixels((frame, 0))  # y値は無視、xだけ使う
        x_pixel = int(pixel_coord[0])

        # 座標が画像範囲内か確認
        if 0 <= x_pixel < self.canvas_width:
            cv2.line(img_bgr, (x_pixel, 0), (x_pixel, self.canvas_height), (0, 0, 255), 1, lineType=cv2.LINE_AA)

        cv2.imshow('Graph', img_bgr)



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
            s = record_data.start_frame
            e = record_data.end_frame
            if s <= frame <= e:
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
        
        return self.current_frame