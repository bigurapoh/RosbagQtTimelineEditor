def frames_to_timecode(frame: int, fps: int) -> str:
    """
    Convert a frame number to a timecode string in HH:MM:SS:FF format.

    :param frame: Frame index (0-based).
    :param fps: Frames per second.
    :return: Timecode string.
    """
    if fps <= 0:
        raise ValueError("FPS must be a positive integer")

    hours = frame // (fps * 3600)
    remainder = frame % (fps * 3600)
    minutes = remainder // (fps * 60)
    remainder = remainder % (fps * 60)
    seconds = remainder // fps
    frames = remainder % fps

    return f"{hours:02d}:{minutes:02d}:{seconds:02d}:{frames:02d}"


def timecode_to_frames(timecode: str, fps: int) -> int:
    """
    Convert a timecode string HH:MM:SS:FF to a frame number.

    :param timecode: Timecode string in HH:MM:SS:FF format.
    :param fps: Frames per second.
    :return: Frame index (0-based).
    """
    parts = timecode.split(':')
    if len(parts) != 4:
        raise ValueError("Timecode must be in HH:MM:SS:FF format")
    hours, minutes, seconds, frames = map(int, parts)
    total_frames = (
        hours * 3600 + minutes * 60 + seconds
    ) * fps + frames
    return total_frames
