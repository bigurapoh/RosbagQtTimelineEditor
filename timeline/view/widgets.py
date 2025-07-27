# view/widgets.py

from PySide6.QtWidgets import (
    QWidget, QPushButton, QSlider, QGraphicsView,
    QVBoxLayout, QHBoxLayout, QFileDialog, QLabel
)
from PySide6.QtCore import Qt, Slot
from controller.timeline_controller import TimelineController
from model.loaders import CsvLoader, JsonLoader  # 必要に応じて他フォーマットも追加
from model.data import TrackData
from theme import get_theme

class TimelineWidget(QWidget):
    def __init__(self, theme, parent=None):
        super().__init__(parent)

        self.theme = get_theme(theme)

        # --- UI構築 ---
        self.play_button = QPushButton("Play")
        self.pause_button = QPushButton("Pause")
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(10, 200)
        self.zoom_slider.setValue(100)
        self.load_button = QPushButton("Load Data")
        self.frame_label = QLabel("Frame: 0")

        self.view = QGraphicsView()  # 実際のタイムライン表示用
        self.view.setScene(None)

        controls = QHBoxLayout()
        controls.addWidget(self.play_button)
        controls.addWidget(self.pause_button)
        controls.addWidget(QLabel("Zoom"))
        controls.addWidget(self.zoom_slider)
        controls.addWidget(self.load_button)
        controls.addWidget(self.frame_label)

        layout = QVBoxLayout(self)
        layout.addWidget(self.view)
        layout.addLayout(controls)
        self.setLayout(layout)

        # --- モデル／コントローラの初期化 ---
        self.controller = TimelineController(graphics_view=self.view, theme=self.theme)

        # --- シグナル／スロット接続 (View -> Controller) ---
        self.load_button.clicked.connect(self.on_load_button_clicked)
        self.play_button.clicked.connect(self.controller.play)
        self.pause_button.clicked.connect(self.controller.pause)
        self.zoom_slider.valueChanged.connect(self.on_zoom_changed)

        # --- コントローラからの更新を受け取る ---
        self.controller.data_loaded.connect(self.on_data_loaded)
        self.controller.frame_changed.connect(self.on_frame_changed)
        self.controller.zoom_changed.connect(self.on_zoom_update)

        self._script_tracks = []

    @Slot()
    def on_load_button_clicked(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select timeline data file", "",
            "Data Files (*.csv *.json *.xml);;All Files (*)"
        )
        if not path:
            return

        ext = path.lower().rsplit(".", 1)[-1]
        if ext == "csv":
            loader = CsvLoader()
        elif ext == "json":
            loader = JsonLoader()
        else:
            return  # 未対応

        self.controller.load_data(path, loader)

    @Slot(int)
    def on_zoom_changed(self, value: int):
        zoom_factor = value / 100.0
        self.controller.set_zoom(zoom_factor)

    @Slot(list)
    def on_data_loaded(self, tracks):
        self.frame_label.setText("Frame: 0")

    @Slot(int)
    def on_frame_changed(self, frame_index):
        self.frame_label.setText(f"Frame: {frame_index}")

    @Slot(float)
    def on_zoom_update(self, zoom_factor):
        self.view.resetTransform()
        self.view.scale(zoom_factor, 1.0)
    
    def add_track(self, track: TrackData):
        self._script_tracks.append(track)

    def update_layout(self):
        self.controller.load_tracks(self._script_tracks)


class TimelineView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.timeline = TimelineWidget(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.timeline)
        self.setLayout(layout)
