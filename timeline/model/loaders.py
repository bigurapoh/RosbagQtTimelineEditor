# model/loaders.py

from abc import ABC, abstractmethod
from typing import List, Dict
import csv
import json
import xml.etree.ElementTree as ET

from model.data import TrackData, ClipData


class DataLoader(ABC):
    """
    データ読み込みの共通インターフェース。
    各フォーマットごとに load(path:str) -> List[TrackData] を実装する。
    """
    @abstractmethod
    def load(self, path: str) -> List[TrackData]:
        ...

#!--- 10.Csvだけ一応Mockに使えるかも知れないので残してxmlとjson loaderは削除. そしてRosbagLodaerの追加

class CsvLoader(DataLoader):
    """
    CSVから
    track_id,start_frame,end_frame,その他メタデータ...
    の形式を想定して読み込むローダー。
    """
    def load(self, path: str) -> List[TrackData]:
        tracks: Dict[int, TrackData] = {}

        with open(path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    tid = int(row['track_id'])
                    start = int(row['start_frame'])
                    end = int(row['end_frame'])
                except KeyError as e:
                    raise ValueError(f"CSVに必須カラムがありません: {e}")
                except ValueError as e:
                    raise ValueError(f"数値変換エラー: {e}")

                # メタデータとして残りのフィールドを格納
                meta = {
                    k: v for k, v in row.items()
                    if k not in ('track_id', 'start_frame', 'end_frame')
                }

                # 終端 frame から duration を計算
                duration = end - start + 1
                clip = ClipData(
                    title=meta.get('title', ''),
                    start_frame=start,
                    duration_frames=duration,
                    track_id=tid,
                    metadata=meta
                )

                if tid not in tracks:
                    tracks[tid] = TrackData(id=tid)
                tracks[tid].add_clip(clip)

        return list(tracks.values())


class JsonLoader(DataLoader):
    """
    JSON構造は以下を想定:
    [
      {
        "track_id": 1,
        "clips": [
           {"start_frame":0, "end_frame":24, "metadata": {...}},
           ...
        ]
      },
      ...
    ]
    """
    def load(self, path: str) -> List[TrackData]:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)

        tracks: List[TrackData] = []
        for entry in data:
            try:
                tid = int(entry['track_id'])
                clips = entry.get('clips', [])
            except KeyError as e:
                raise ValueError(f"JSONに必須キーがありません: {e}")

            track = TrackData(id=tid)
            for c in clips:
                try:
                    start = int(c['start_frame'])
                    end = int(c['end_frame'])
                except KeyError as e:
                    raise ValueError(f"クリップデータに必須キーがありません: {e}")
                meta = c.get('metadata', {})

                # 終端 frame から duration を計算
                duration = end - start + 1
                clip = ClipData(
                    title=meta.get('title', ''),
                    start_frame=start,
                    duration_frames=duration,
                    track_id=tid,
                    metadata=meta
                )
                track.add_clip(clip)

            tracks.append(track)

        return tracks


class XmlLoader(DataLoader):
    """
    XML構造は以下を想定:
    <Timeline>
      <Track id="1">
         <Clip start_frame="0" end_frame="24">
            <Metadata key="foo">bar</Metadata>
            ...
         </Clip>
         ...
      </Track>
      ...
    </Timeline>
    """
    def load(self, path: str) -> List[TrackData]:
        tree = ET.parse(path)
        root = tree.getroot()

        tracks: List[TrackData] = []
        for t_elem in root.findall('Track'):
            tid = int(t_elem.get('id', '-1'))
            if tid < 0:
                raise ValueError("Track要素に有効なid属性がありません")

            track = TrackData(id=tid)
            for c_elem in t_elem.findall('Clip'):
                try:
                    start = int(c_elem.get('start_frame'))
                    end = int(c_elem.get('end_frame'))
                except (TypeError, ValueError):
                    raise ValueError("Clip要素にstart_frame/end_frame属性が正しく設定されていません")

                # Metadata要素を辞書化
                meta: Dict[str, str] = {}
                for m in c_elem.findall('Metadata'):
                    key = m.get('key')
                    if key is not None:
                        meta[key] = m.text or ""

                # 終端 frame から duration を計算
                duration = end - start + 1
                clip = ClipData(
                    title=meta.get('title', ''),
                    start_frame=start,
                    duration_frames=duration,
                    track_id=tid,
                    metadata=meta
                )
                track.add_clip(clip)

            tracks.append(track)

        return tracks
