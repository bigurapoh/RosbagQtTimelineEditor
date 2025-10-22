# model/loaders.py

from abc import ABC, abstractmethod
from typing import Dict
import csv
import rosbag
from model.data import TrackMetaData, ClipData, RecordData, MockData, RosbagData


class DataLoader(ABC):
    """
    データ読み込みの共通インターフェース。
    各フォーマットごとに load(path:str) -> List[TrackData] を実装する。
    """
    loader_fps:int = 100
    @abstractmethod
    def load(self, path: str) -> RecordData:
        ...

# CsvはMock用に使うかもしれない

class CsvLoader(DataLoader):
    """
    CSVから
    track_id,start_frame,end_frame,その他メタデータ...
    の形式を想定して読み込むローダー。
    """
    def load(self, path: str) -> MockData:
        tracks: Dict[int, TrackMetaData] = {}

        with open(path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    name = row['name']
                    start = int(row['start_frame'])
                    end = int(row['end_frame'])
                except KeyError as e:
                    raise ValueError(f"CSVに必須カラムがありません: {e}")
                except ValueError as e:
                    raise ValueError(f"数値変換エラー: {e}")

                # メタデータとして残りのフィールドを格納
                meta = {
                    k: v for k, v in row.items()
                    if k not in ('name', 'start_frame', 'end_frame')
                }

                # 終端 frame から duration を計算
                duration = end - start + 1
                clip = ClipData(
                    title=name,
                    start_frame=start,
                    duration_frames=duration,
                    metadata=meta
                )

                if name not in tracks:
                    tracks[name] = TrackMetaData(name=name, is_movable=False)
                tracks[name].add_clip(clip)
        
        record_data = MockData(track_metadatas=list(tracks.values()), fps=self.loader_fps)
        return record_data

def _msg_time_to_sec(t):
	# rospy.Time互換のオブジェクトをfloat秒へ
	# (t.secs, t.nsecs) を持つ前提
	return float(t.secs) + float(t.nsecs) * 1e-9

# pathからRecordDataを作成する
class RosbagLoader(DataLoader):
    def load(self, path:str) -> RosbagData:

        # Bagファイルを読み込み
        bag = rosbag.Bag(path)
        topic_dict = {} # {'tpc0':[(time, msg), ...], 'tpc1':[...], ...}
        for info in bag.read_messages():
            topic, msg, t = info
            if not topic in topic_dict.keys():
                topic_dict[topic] = []
            topic_dict[topic].append((t, msg))
        bag.close()
        
        
        # 複数トピックにまたがるstart_timeを計算
        start_times = [_msg_time_to_sec(v[0][0]) for v in topic_dict.values()]
        start_time = min(start_times)

        """
        RosBagDataにはtrack_metadatas及びtrack_records(及びfps)が必要
        track_metadatas => List[TrackMetaData]、特に .name="topic名", .clips[0].start_frame, .clips[0].end_frameが重要(RosbagDataではclipsは長さ1で固定) 
        track_records => {"tpc1": (record1, topic_type), "tpc2": (record2, topic_type2), ...}という形式のdictを作成(record: {frame0: msg0, frame1: msg1, ...})
        """
        # 各トピックごとにclipdata及びtrackmetadataを作成
        track_records = {}
        track_metadatas = []
        for topic_name, v in topic_dict.items():

            raw_record = {}
            # start_timeからの経過時間を各msgごとに計算して, frameに変換 :future: 複数rosbagファイル対応できるようにstart_timeを工夫すべき
            for t, msg in v:
                curr_time = _msg_time_to_sec(t)
                frame = int((curr_time - start_time) * self.loader_fps) # :check:
                raw_record[frame] = msg

            # start_frame及びend_frameを計算, ClipDataをtitle=""で作成
            local_start_frame = min(raw_record.keys()) # なお、start_time変数はグローバルなminであるのに対して、これはlocal、つまりこのトピック内のみで最小
            local_end_frame = max(raw_record.keys())
            duration_frames = local_end_frame - local_start_frame + 1
            clip_data = ClipData(title=topic_name, start_frame=local_start_frame, duration_frames=duration_frames)
            
            # topic_typeの取得
            topic_type = type(v[0][1])

            # track_recordsへの登録
            track_records[topic_name] = (raw_record, topic_type)

            # TrackMetaDataをname=topic名で作成, add_clip
            track_metadata = TrackMetaData(name=topic_name, ui_height=60)
            track_metadata.add_clip(clip_data)

            track_metadatas.append(track_metadata)
        
        return RosbagData(track_metadatas, track_records, self.loader_fps)
