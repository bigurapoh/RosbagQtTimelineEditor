# view/widgets.py

from PySide6.QtWidgets import (
    QWidget, QPushButton, QSlider, QGraphicsView,
    QVBoxLayout, QHBoxLayout, QFileDialog, QLabel, QGraphicsScene
)
from PySide6.QtCore import Qt, Slot

from controller.timeline_controller import TimelineController
from model.loaders import CsvLoader, JsonLoader, XmlLoader, DataLoader
from model.data import TrackMetaData, ClipData, RecordData
from theme import get_theme

# import all of your Item classes
from view.items import (
    TimeLabelItem, RulerItem,
    PlayheadTriangleItem, PlayheadLineItem, EndLineItem,
    TrackHeaderItem, TrackLaneItem, ClipItem
)

class TimelineWidget(QWidget):
    def __init__(self, theme="dark", parent=None):
        super().__init__(parent)
        self.theme = get_theme(theme)


        #!-- 1. eventの発火を原則controllerに通知する(例外はload data)

        # --- toolbar ---
        tb = QWidget()
        tb_lay = QHBoxLayout(tb)
        for w in (
            (QPushButton("Play"), self.on_play),
            (QPushButton("Pause"), self.on_pause),
            (QPushButton("<<"), lambda: self._step(-1)),
            (QPushButton(">>"), lambda: self._step(+1)),
        ):
            btn, cb = w
            btn.clicked.connect(cb)
            tb_lay.addWidget(btn)

        self.hZoom = QSlider(Qt.Horizontal)
        self.hZoom.setRange(1,100)
        self.hZoom.setValue(1)
        self.hZoom.valueChanged.connect(self.on_hzoom)
        tb_lay.addWidget(QLabel("H-Zoom:"))
        tb_lay.addWidget(self.hZoom)

        # --- graphics view & scene ---
        self.view  = QGraphicsView()
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)

        # connect controller
        self.controller = TimelineController()
        self.controller.data_loaded.connect(self._on_data_loaded)
        self.controller.frame_changed.connect(self._on_frame_changed)
        self.controller.zoom_changed.connect(self._on_zoom_changed)

        # --- layout ---
        main = QVBoxLayout(self)
        main.addWidget(tb)
        main.addWidget(self.view)
        self.setLayout(main)

        # dynamic lists
        self._headers = []
        self._lanes   = []
        self._clips   = []

        # static items placeholders
        self._timeLabel      = None
        self._ruler          = None
        self._playTri        = None
        self._playLine       = None
        self._endLine        = None

    @Slot()
    def on_play(self):
        self.controller.play()

    @Slot()
    def on_pause(self):
        self.controller.pause()

    @Slot(int)
    def _step(self, delta):
        self._on_frame_changed(self._current_frame + delta)

    @Slot(int)
    def on_hzoom(self, val):
        z = 0.5 + (val/100.0)*3.5
        self.controller.set_zoom(z)

    #!--- 2. add_trackを多態性を持ったメソッドに変更。あとcsv, json, xmlは正直いらないのでなくすことを検討.
    @Slot()
    def on_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select data", "", 
            "Data Files (*.csv *.json *.xml)"
        )
        # if not path: return
        # ext = path.rsplit(".",1)[-1].lower()
        # loader = {"csv":CsvLoader,"json":JsonLoader,"xml":XmlLoader}.get(ext)()
        # self.controller.load_data(path, loader)
    
    def add_record(self, path=None, recorddata:RecordData=None):
        if path is not None:
            # if not path: return
            ext = path.rsplit(".",1)[-1].lower()
            loader:DataLoader = {"csv":CsvLoader,"json":JsonLoader,"xml":XmlLoader}.get(ext)()
            self.controller.load_data(path, loader)
            record_data = loader.load(path)
        
        if recorddata is not None:
            self.controller.load_data()

    
    #!--- 3. ここらへんのメソッドがmodelのobserverから_update_sceneを発火するだけで置換できるか検討。またロジックをcontrollerなどに移す
    @Slot(list)
    def _on_data_loaded(self, tracks):
        # keeps the model
        self._tracks = tracks
        # reset frame
        self._current_frame = 0
        # redraw everything
        self._update_scene()

    @Slot(int)
    def _on_frame_changed(self, frame):
        self._current_frame = frame
        # update time-label
        self._timeLabel.updateTime(frame)
        # redraw static parts & clips
        self._update_scene()

    @Slot(float)
    def _on_zoom_changed(self, zoom):
        self._h_zoom = zoom
        self._update_scene()

    def _update_scene(self):
        """ exactly the same as QtEditorialTimelineWidget.updateLayout() :contentReference[oaicite:5]{index=5} """
        # 1) clear old dynamic items
        for lst in (self._headers, self._lanes, self._clips):
            for it in lst:
                self.scene.removeItem(it)
            lst.clear()
        # then entirely clear static items too
        for it in (self._timeLabel, self._ruler, self._playTri, self._playLine, self._endLine):
            if it:
                self.scene.removeItem(it)

        # 2) scene rect
        scene_w = max(2000, (self._tracks and 
            max(c.start_frame+c.duration_frames for t in self._tracks for c in t.clips)*self.theme["BASE_PIXELS_PER_FRAME"]*self._h_zoom + self.theme["LEFT_MARGIN"]
        ))
        y = self.theme["TOP_MARGIN"]
        for t in self._tracks:
            y += t.height * self._v_zoom + self.theme["TRACK_SPACING"]
        scene_h = y + self.theme["BOTTOM_MARGIN"]
        self.scene.setSceneRect(0,0, scene_w, scene_h)

        # 3) static items
        self._timeLabel = TimeLabelItem(self._current_frame, self.theme)
        self.scene.addItem(self._timeLabel)
        self._ruler     = RulerItem(self.theme)
        self._ruler.setPos(self.theme["LEFT_MARGIN"],0)
        self.scene.addItem(self._ruler)
        # playhead
        phx = self.theme["LEFT_MARGIN"] + self._current_frame*self.theme["BASE_PIXELS_PER_FRAME"]*self._h_zoom
        self._playLine  = PlayheadLineItem(self, self.theme)
        self._playLine .setPos(phx, self.theme["TOP_MARGIN"])
        self.scene.addItem(self._playLine )
        self._playTri   = PlayheadTriangleItem(self, self.theme)
        self._playTri  .setPos(phx, 0)
        self.scene.addItem(self._playTri  )
        # end-line
        #!--- 12.end_frameの値はcontrollerがmodelに渡す、current_frameの値をend_frameが超えたらcurrent_frameの値を初期化してviewに発火
        #!---    end_frameの初期値であるini_end_frameはmodelによって規定される値だが、end_frame自体はviewで設定する値
        min_end = max((c.start_frame+c.duration_frames) for t in self._tracks for c in t.clips)
        end_frame = max(min_end+24, 100)
        ex = self.theme["LEFT_MARGIN"] + end_frame*self.theme["BASE_PIXELS_PER_FRAME"]*self._h_zoom
        self._endLine   = EndLineItem(self, self.theme)
        self._endLine  .setPos(ex, self.theme["TOP_MARGIN"])
        self.scene.addItem(self._endLine  )

        # 4) dynamic items
        cy = self.theme["TOP_MARGIN"]
        for t in self._tracks:
            # headers
            hdr = TrackHeaderItem(t, self.theme)
            hdr.setPos(0, cy)
            self.scene.addItem(hdr)
            self._headers.append(hdr)
            # lanes
            lane = TrackLaneItem(t, self.theme)
            lane.setGeometry(self.theme["LEFT_MARGIN"], cy,
                             scene_w-self.theme["LEFT_MARGIN"],
                             t.height*self._v_zoom)
            self.scene.addItem(lane)
            self._lanes.append(lane)
            # clips
            for c in t.clips:
                cx = self.theme["LEFT_MARGIN"] + c.start_frame*self.theme["BASE_PIXELS_PER_FRAME"]*self._h_zoom
                cw = ((c.duration_frames*self.theme["BASE_PIXELS_PER_FRAME"])+self.theme["BASE_PIXELS_PER_FRAME"])*self._h_zoom
                ch = t.height*self._v_zoom
                ci = ClipItem(c, t, self.theme)
                ci.setGeometry(cx, cy, cw, ch)
                self.scene.addItem(ci)
                self._clips.append(ci)
            cy += t.height*self._v_zoom + self.theme["TRACK_SPACING"]

        # finally, force a redraw
        self.view.viewport().update()
