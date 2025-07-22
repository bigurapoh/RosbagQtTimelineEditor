from typing import Dict, Any

# --- Default Constants (can be overridden by theme) ---
DEFAULT_CONSTANTS: Dict[str, Any] = {
    "LEFT_MARGIN": 150,
    "TOP_MARGIN": 30,
    "BOTTOM_MARGIN": 20,
    "TRACK_SPACING": 2,
    "BASE_PIXELS_PER_FRAME": 10,
    "DEFAULT_TRACK_HEIGHT": 60,
}

# --- Default Themes ---
THEMES: Dict[str, Dict[str, Any]] = {
    "dark": {
        "timeLabel_bg": "#141414",
        "timeLabel_text": "#FFFFFF",
        "ruler_bg": "#1E1E1E",
        "ruler_tick_major": "#FFFFFF",
        "ruler_tick_minor": "#808080",
        "playhead_color": "#FFA500",
        "track_header_bg": "#282828",
        "track_header_text": "#FFFFFF",
        "track_lane_bg1": "#323232",
        "track_lane_bg2": "#3E3E3E",
        "track_lane_border": "#505050",
        "clip_fill": "#6496C8",
        "clip_fill_selected": "#96C8FF",
        "clip_border": "#000000",
        "end_line_color": "#C83232",
        "background_color": "#111111",
    },
    "light": {
        "timeLabel_bg": "#F0F0F0",
        "timeLabel_text": "#000000",
        "ruler_bg": "#E0E0E0",
        "ruler_tick_major": "#000000",
        "ruler_tick_minor": "#808080",
        "playhead_color": "#FF8C00",
        "track_header_bg": "#D0D0D0",
        "track_header_text": "#000000",
        "track_lane_bg1": "#E8E8E8",
        "track_lane_bg2": "#F0F0F0",
        "track_lane_border": "#A0A0A0",
        "clip_fill": "#90CAF9",
        "clip_fill_selected": "#64B5F6",
        "clip_border": "#000000",
        "end_line_color": "#E53935",
        "background_color": "#FFFFFF",
    },
}


def get_theme(config: Any = None) -> Dict[str, Any]:
    """
    Return a complete theme dictionary.
    If config is a string matching one of the keys in THEMES, that theme is used.
    If config is a dict, it is merged over the dark theme defaults.
    Otherwise, defaults to the dark theme.
    """
    if isinstance(config, str) and config in THEMES:
        base = THEMES[config].copy()
    elif isinstance(config, dict):
        base = THEMES["dark"].copy()
        base.update(config)
    else:
        base = THEMES["dark"].copy()
    full_theme = DEFAULT_CONSTANTS.copy()
    full_theme.update(base)
    return full_theme
