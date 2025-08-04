# view/widgets.py

from PySide6.QtWidgets import (
    QWidget, QPushButton, QSlider, QGraphicsView,
    QVBoxLayout, QHBoxLayout, QFileDialog, QLabel, QGraphicsScene
)
from PySide6.QtCore import Qt, Slot, Signal

from controller.timeline_controller import TimelineController
from model.loaders import CsvLoader, RosbagLoader, DataLoader
from model.data import TrackMetaData, ClipData, RecordData, RecordMngModel, TimelineModel
from theme import get_theme

# import all of your Item classes
from view.items import (
    TimeLabelItem, RulerItem, PlayheadItem, EndLineItem,
    TrackHeaderItem, TrackLaneItem, ClipItem
)

from types import SimpleNamespace
from typing import List

def _get_shrink_rate(base_pix_width:float, left_margin):
    shrink_rate_candidate_list = [1, 5, 10, 30, 60, 120]
    
    for shrink_rate in shrink_rate_candidate_list:
        _base_scene_w = int(base_pix_width / shrink_rate) + left_margin
        if _base_scene_w + 100 < 2000:
            return shrink_rate
    
    raise ValueError("読み込んだデータが規定の長さを超えています.")

class TimelineWidget(QWidget):
    pressed_playhead = Signal()
    released_playhead = Signal(int)
    pressed_endline = Signal()
    released_endline = Signal(int)

    class _view_model():
        # frameからitemの位置を計算する+endlineとcurrentine(playhead)の制限を担う
        current_frame = 0
        end_frame = 0
        end_frame_limit :int = 0 # all_records_end_frameとしてmodelでは定義
        shrink_rate = 1
        tracks:List[TrackMetaData] = []
        h_zoom = 1 # controllerに送らずview単体で処理
        v_zoom = 1 # 未実装
        fps = None
        theme = None

        @classmethod
        def _frame_to_pix(cls, frame):
            """offsetをLEFT_MARGIN, 比率をfps,px/ds,r_shrinkによって定義"""
            return cls.h_zoom*int((10*frame/cls.fps)*cls.theme["BASE_PIXELS_PER_DECI_SECOND"]/cls.shrink_rate) + cls.theme["LEFT_MARGIN"]
        
        @classmethod
        def _pix_to_frame(cls, pix_x):
            """_frame_to_pixの逆計算"""
            new_deci_second = ((pix_x - cls.theme["LEFT_MARGIN"])/cls.h_zoom) * cls.shrink_rate / cls.theme["BASE_PIXELS_PER_DECI_SECOND"]
            new_frame = int(new_deci_second*cls.fps/10)
            return new_frame
        
        @classmethod
        def _limit_x_end_line_move(cls, pix_x):
            new_frame = cls._pix_to_frame(pix_x)
            if new_frame < 0: # :future: start_lineの実装
                new_frame = 10
                new_x = cls._frame_to_pix(new_frame)
            elif new_frame > cls.end_frame_limit:
                new_frame = cls.end_frame_limit
                new_x = cls._frame_to_pix(new_frame)
            else:
                new_x = pix_x
            
            return new_x
        
        @classmethod
        def _limit_x_current_line_move(cls, pix_x):
            new_frame = cls._pix_to_frame(pix_x)

            if new_frame < 0: # :future: start_lineの実装
                new_frame = 0
                new_x = cls._frame_to_pix(new_frame)
            elif new_frame > cls.end_frame:
                new_frame = cls.end_frame
                new_x = cls._frame_to_pix(new_frame)
            else:
                new_x = pix_x
            
            return new_x


    def __init__(self, theme="dark", parent=None):
        super().__init__(parent)
        # self.theme = get_theme(theme)
        self._view_model.theme = get_theme(theme)


        #!--- 1. eventの発火を原則controllerに通知する(例外はload data)

        # ToDo: model初期化
        self.record_mng_model:RecordMngModel = RecordMngModel()
        self.timeline_model:TimelineModel = TimelineModel()

        # connect controller
        self.controller = TimelineController(self.record_mng_model, self.timeline_model, self)
        self._view_model.fps = self.controller.loader_fps
        # self.controller.data_loaded.connect(self._on_data_loaded)
        # self.controller.frame_changed.connect(self._on_frame_changed)
        # self.controller.zoom_changed.connect(self._on_zoom_changed)

        # --- toolbar ---
        tb = QWidget()
        tb_lay = QHBoxLayout(tb)
        for w in (
            (QPushButton("Play"), self.controller.play),
            (QPushButton("Pause"), self.controller.pause),
            (QPushButton("<<"), self.controller.go_start_frame),
            (QPushButton(">>"), self.controller.go_end_frame),
        ):
            btn, cb = w
            btn.clicked.connect(cb)
            tb_lay.addWidget(btn)

        self.hZoom = QSlider(Qt.Horizontal)
        self.hZoom.setRange(1,100)
        self.hZoom.setValue(1)
        self.hZoom.valueChanged.connect(self._on_hzoom_changed)
        tb_lay.addWidget(QLabel("H-Zoom:"))
        tb_lay.addWidget(self.hZoom)


        # modelから受け取る値(view model)
        # self._current_frame = 0
        # self._end_frame = None
        # self._end_frame_limit = 0 # all_records_end_frameとしてmodelでは定義
        # self.shrink_rate = 1
        # self._tracks:List[TrackMetaData] = []
        # self._h_zoom = 1 # controllerに送らずview単体で処理
        # self._v_zoom = 1 # 未実装

        # --- その他のSignalをつなぐ ---
        self.released_endline.connect(self.controller.release_end_line)
        self.released_playhead.connect(self.controller.release_current_line)
        self.pressed_endline.connect(self.controller.pause)
        self.pressed_playhead.connect(self.controller.pause)

        # modelの変更通知を受け取る
        self.controller.end_frame_changed.connect(self._on_end_frame_changed)
        self.controller.data_loaded.connect(self._on_data_added)
        self.controller.frame_changed.connect(self._on_frame_changed)

        # --- graphics view & scene ---
        self.view  = QGraphicsView()
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)

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
        self._playHead = None
        # self._playTri        = None
        # self._playLine       = None
        self._endLine        = None

    # @Slot()
    # def on_play(self):
    #     self.controller.play()

    # @Slot()
    # def on_pause(self):
    #     self.controller.pause()

    # @Slot(int)
    # def _step(self, delta):
    #     self._on_frame_changed(self._current_frame + delta)

    # @Slot(int)
    # def on_hzoom(self, val):
    #     z = 0.5 + (val/100.0)*3.5
    #     self.controller.set_zoom(z)

    #!----14. loader_fps, intervalなどの兼ね合いで、add_recordはcontrollerのものを呼び出す形にしてここの実装はなくす ---Done
    #!--- 2. --- Done ---add_trackを多態性を持ったメソッドに変更。あとcsv, json, xmlは正直いらないのでなくすことを検討.--- Done
    # @Slot()
    # def on_load(self):
    #     path, _ = QFileDialog.getOpenFileName(
    #         self, "Select data", "", 
    #         "Data Files (*.csv *.json *.xml)"
    #     )
    #     # if not path: return
    #     # ext = path.rsplit(".",1)[-1].lower()
    #     # loader = {"csv":CsvLoader,"json":JsonLoader,"xml":XmlLoader}.get(ext)()
    #     # self.controller.load_data(path, loader)
    
    # def add_record(self, path=None, record_data:RecordData=None):
    #     if path is not None:
    #         # if not path: return
    #         ext = path.rsplit(".",1)[-1].lower()
    #         loader:DataLoader = {"csv":CsvLoader,"bag":RosbagLoader}.get(ext)()
    #         loaded_record_data = loader.load(path)
    #         self.controller.add_record(loaded_record_data)
    #         # self.controller.load_data(path, loader)
    #         return

    #     if record_data is not None:
    #         self.controller.add_record(record_data)

    
    #!--- 3. ここらへんのメソッドがmodelのobserverから_update_sceneを発火するだけで置換できるか検討。またロジックをcontrollerなどに移す
    #-- modelからの通知を受け取ってviewを更新する
    # (loadに時間がかかるようならon_loadingメソッドの追加を検討)
    @Slot()
    def _on_data_added(self):
        # track等が追加された際の更新
        self._view_model.tracks = self.record_mng_model.all_track_metadatas
        self._view_model.end_frame_limit = self.record_mng_model.all_records_end_frame # 実際のデータの長さからsceneのwidthを規定(end lineとは関係しない)
        # self._view_model.end_frame = self.timeline_model.end_frame # end_frameは初期値がモデル依存なのでmodel管理の値とする
        
        # adaptive_pixels_per_frameを設定
        base_pix_width_float = (10*self._view_model.end_frame_limit/self._view_model.fps)*self._view_model.theme["BASE_PIXELS_PER_DECI_SECOND"]
        self._view_model.shrink_rate = _get_shrink_rate(base_pix_width_float, self._view_model.theme["LEFT_MARGIN"])
        
        self._update_scene()
    
    @Slot()
    def _on_load_button_clicked(self):
        # :Future:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select data", "", 
            "Data Files (*.csv *.bag)"
        )
        # if not path: return
        # ext = path.rsplit(".",1)[-1].lower()
        # loader = {"csv":CsvLoader,"json":JsonLoader,"xml":XmlLoader}.get(ext)()
        # self.controller.load_data(path, loader)
    
    @Slot()
    def _on_end_frame_changed(self):
        self._view_model.end_frame = self.timeline_model.end_frame
    
    @Slot()
    def _on_frame_changed(self):
        # timemodelの更新
        self._view_model.current_frame = self.timeline_model.current_frame
        self._update_scene()
    
    @Slot(float)
    def _on_hzoom_changed(self, val):
        zoom = 0.5 + (val/100.0)*3.5
        # zoomの更新を直接view内から受け取る

        # ToDo: self.viewのscaleに関する処理を行う (↓参考)
        # self.zoom_factor = zoom
        # self.graphics_view.resetTransform()
        # self.graphics_view.scale(self.zoom_factor, 1.0)
        # self.zoom_changed.emit(self.zoom_factor)
        
        self._view_model.h_zoom = zoom
        self._update_scene()
    
    # @Slot(list)
    # def _on_data_loaded(self, tracks):
    #     # keeps the model
    #     self._tracks = tracks
    #     # reset frame
    #     self._current_frame = 0
    #     # redraw everything
    #     self._update_scene()

    # @Slot(int)
    # def _on_frame_changed(self, frame):
    #     self._current_frame = frame
    #     # update time-label
    #     self._timeLabel.updateTime(frame)
    #     # redraw static parts & clips
    #     self._update_scene()
    # def _frame_to_pix(self, frame):
    #     """offsetをLEFT_MARGIN, 比率をfps,px/ds,r_shrinkによって定義"""
    #     return self._view_model.h_zoom*int((10*frame/self.controller.loader_fps)*self._view_model.theme["BASE_PIXELS_PER_DECI_SECOND"]/self._view_model.shrink_rate) + self._view_model.theme["LEFT_MARGIN"]

    def _update_scene(self):
        """ exactly the same as QtEditorialTimelineWidget.updateLayout() :contentReference[oaicite:5]{index=5} """
        # 1) clear old dynamic items
        for lst in (self._headers, self._lanes, self._clips):
            for it in lst:
                self.scene.removeItem(it)
            lst.clear()
        # then entirely clear static items too
        for it in (self._timeLabel, self._ruler, self._playHead, self._endLine):
            if it:
                self.scene.removeItem(it)

        # 2) scene rect
        _base_scene_w = self._view_model._frame_to_pix(self._view_model.end_frame_limit)
        # _base_scene_w = self._end_frame_limit*self.theme["BASE_PIXELS_PER_DECI_SECOND"]*self._h_zoom + self.theme["LEFT_MARGIN"]
        scene_w = max(1500, _base_scene_w)
        # scene_w = max(2000, (self._tracks and 
        #     max(c.start_frame+c.duration_frames for t in self._tracks for c in t.clips)*self.theme["BASE_PIXELS_PER_DECI_SECOND"]*self._h_zoom + self.theme["LEFT_MARGIN"]
        # ))
        y = self._view_model.theme["TOP_MARGIN"]
        for t in self._view_model.tracks:
            y += t.height * self._view_model.v_zoom + self._view_model.theme["TRACK_SPACING"]
        scene_h = y + self._view_model.theme["BOTTOM_MARGIN"]
        self.scene.setSceneRect(0,0, scene_w, scene_h)

        # 3) static items
        # view_model = SimpleNamespace(**{'fps':self.controller.loader_fps, 's_rate':self._view_model.shrink_rate, 'ef_limit':self._view_model.end_frame_limit})
        #timelabel
        self._timeLabel = TimeLabelItem(self._view_model.current_frame, self._view_model.theme)
        self.scene.addItem(self._timeLabel)
        
        # ruler
        self._ruler     = RulerItem(self._view_model)
        self._ruler.setPos(self._view_model.theme["LEFT_MARGIN"],0)
        self.scene.addItem(self._ruler)
        
        # end-line
        #!--- 12.end_frameの値はcontrollerがmodelに渡す、current_frameの値をend_frameが超えたらcurrent_frameの値を初期化してviewに発火
        #!---    end_frameの初期値であるini_end_frameはmodelによって規定される値だが、end_frame自体はviewで設定する値
        # min_end = max((c.start_frame+c.duration_frames) for t in self._tracks for c in t.clips)
        # end_frame = max(min_end+24, 100)
        ex = self._view_model._frame_to_pix(self._view_model.end_frame)
        print("update_scene"+"-*"*20)
        # ex = self.theme["LEFT_MARGIN"] + self._end_frame*self.theme["BASE_PIXELS_PER_DECI_SECOND"]*self._h_zoom
        self._endLine   = EndLineItem(self, self._view_model.theme, self._view_model)
        self._endLine  .setPos(ex, self._view_model.theme["TOP_MARGIN"])
        self.scene.addItem(self._endLine  )
        
        # playhead
        phx = self._view_model._frame_to_pix(self._view_model.current_frame)
        # phx = self.theme["LEFT_MARGIN"] + self._current_frame*self.theme["BASE_PIXELS_PER_DECI_SECOND"]*self._h_zoom
        self._playHead = PlayheadItem(self, self._view_model.theme, self._view_model)
        self._playHead  .setPos(phx, 0)
        # self._playLine  = PlayheadLineItem(self, self._view_model.theme, self._view_model)
        # self._playLine .setPos(phx, self._view_model.theme["TOP_MARGIN"])
        # self.scene.addItem(self._playLine )
        # self._playTri   = PlayheadTriangleItem(self, self._view_model.theme, self._view_model)
        # self._playTri  .setPos(phx, 0)
        self.scene.addItem(self._playHead)

        # 4) dynamic items
        cy = self._view_model.theme["TOP_MARGIN"]
        for t in self._view_model.tracks:
            # headers
            hdr = TrackHeaderItem(t, self._view_model)
            hdr.setPos(0, cy)
            self.scene.addItem(hdr)
            self._headers.append(hdr)
            # lanes
            lane = TrackLaneItem(t, self._view_model.theme)
            lane.setGeometry(self._view_model.theme["LEFT_MARGIN"], cy,
                             scene_w-self._view_model.theme["LEFT_MARGIN"],
                             t.height*self._view_model.v_zoom)
            self.scene.addItem(lane)
            self._lanes.append(lane)
            # clips
            for c in t.clips:
                cx = self._view_model._frame_to_pix(c.start_frame)
                cw = self._view_model._frame_to_pix(c.end_frame) - cx
                # cx = self.theme["LEFT_MARGIN"] + c.start_frame*self.theme["BASE_PIXELS_PER_DECI_SECOND"]*self._h_zoom
                # cw = ((c.duration_frames*self.theme["BASE_PIXELS_PER_DECI_SECOND"])+self.theme["BASE_PIXELS_PER_DECI_SECOND"])*self._h_zoom # :Check:何故かs+d-1ではなくs+d+1になっている
                ch = t.height*self._view_model.v_zoom
                ci = ClipItem(c, t, self._view_model, t.is_movable)
                ci.setGeometry(cx, cy, cw, ch)
                self.scene.addItem(ci)
                self._clips.append(ci)
            cy += t.height*self._view_model.v_zoom + self._view_model.theme["TRACK_SPACING"]

        # finally, force a redraw
        self.view.viewport().update()
