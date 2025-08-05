# model/loaders.py

from abc import ABC, abstractmethod
from typing import Dict
import csv

from model.data import TrackMetaData, ClipData, RecordData, MockData, RosbagData


class DataLoader(ABC):
    """
    データ読み込みの共通インターフェース。
    各フォーマットごとに load(path:str) -> List[TrackData] を実装する。
    """
    loader_fps:int = 24
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


# pathからRecordDataを作成する
class RosbagLoader(DataLoader):
    def load(self, path:str) -> RosbagData:
        pass
