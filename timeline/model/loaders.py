from __future__ import annotations
import abc
import csv
import json
from typing import List
from .data import TrackData, ClipData


class DataLoader(abc.ABC):
    """
    Abstract base class for loading timeline data from various formats.
    """

    @abc.abstractmethod
    def load(self, path: str) -> List[TrackData]:
        """
        Load timeline data from the given file path.
        Returns a list of TrackData objects.
        """
        pass


class CsvLoader(DataLoader):
    """
    Loads timeline clips from a CSV file with columns:
      track_name, start_frame, end_frame, [metadata...]
    """

    def load(self, path: str) -> List[TrackData]:
        tracks: dict[str, TrackData] = {}
        with open(path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                track_name = row['track_name']
                start_frame = int(row['start_frame'])
                end_frame = int(row['end_frame'])
                metadata = {k: v for k, v in row.items() if k not in ('track_name', 'start_frame', 'end_frame')}
                clip = ClipData(start_frame, end_frame, track_name, metadata)
                if track_name not in tracks:
                    tracks[track_name] = TrackData(track_name)
                tracks[track_name].add_clip(clip)
        return list(tracks.values())


class JsonLoader(DataLoader):
    """
    Loads timeline clips from a JSON file with structure:
      {
        "clips": [
          {"track_name": str, "start_frame": int, "end_frame": int, "metadata": {...}},
          ...
        ]
      }
    """

    def load(self, path: str) -> List[TrackData]:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)

        tracks: dict[str, TrackData] = {}
        for item in data.get('clips', []):
            track_name = item['track_name']
            clip = ClipData(
                start_frame=item['start_frame'],
                end_frame=item['end_frame'],
                track_name=track_name,
                metadata=item.get('metadata', {})
            )
            if track_name not in tracks:
                tracks[track_name] = TrackData(track_name)
            tracks[track_name].add_clip(clip)
        return list(tracks.values())
