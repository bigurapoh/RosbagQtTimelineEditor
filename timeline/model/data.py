from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, List

@dataclass
class ClipData:
    """
    Represents a single clip segment within a track.
    """
    start_frame: int
    end_frame: int
    track_name: str
    metadata: Dict[str, Any] = field(default_factory=dict)

    def duration(self) -> int:
        """
        Returns the duration of this clip in frames.
        """
        return max(0, self.end_frame - self.start_frame)

@dataclass
class TrackData:
    """
    Holds all clips for a given timeline track.
    """
    name: str
    clips: List[ClipData] = field(default_factory=list)

    def add_clip(self, clip: ClipData) -> None:
        """
        Adds a ClipData to this track, ensuring chronological order.
        """
        self.clips.append(clip)
        # Keep sorted by start_frame
        self.clips.sort(key=lambda c: c.start_frame)

    def total_duration(self) -> int:
        """
        Returns total span from earliest start to latest end.
        """
        if not self.clips:
            return 0
        return max(c.end_frame for c in self.clips) - min(c.start_frame for c in self.clips)
