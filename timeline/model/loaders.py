# model/loaders.py

from abc import ABC, abstractmethod
from typing import Dict
import csv
import rosbag
import rospy
from model.data import TrackMetaData, ClipData, RecordData, AnimeGraphData, MockData, RosbagData, RosbagWithBlurData

import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas


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

class DistanceDataLoader(DataLoader):
    target_columns = ['dn2', 'dn1', 'd0', 'd1', 'd2']
    window_radius = 2

    def _frame_to_bgr(self, fig):
        canvas = FigureCanvas(fig)
        canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        rgb = buf.reshape(h, w, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return bgr

    def _make_frame_image(self, data, ylim, goal_idx=None, fig_size=(6,4), dpi=120):
        """
        data: {"distances": list[float], "sg_idx": int, "cl_idx": int}
        goal_idx: int | None  （指定時、goal_idx-1/goal_idxで点が減っても位置は固定）
        """
        distances = data["distances"]
        cl_idx = int(data["cl_idx"])
        sg_rel = int(data["sg_idx"]) - cl_idx              # cl_idxからの相対位置
        # 表示用の相対x（中央0がcl_idx）
        x_rel = np.arange(-self.window_radius, self.window_radius+1)       # 例: self.window_radius=2 -> [-2,-1,0,1,2]
        # 相対→絶対のラベル（見た目の横軸は地点＝絶対インデックス）
        x_abs_labels = cl_idx + x_rel
    
        # まずNaNで全窓を用意（欠ける点はNaNのまま）
        y = np.full_like(x_rel, np.nan, dtype=float)
    
        # distances が始まる絶対index（通常 cl_idx - self.window_radius）
        start_abs = cl_idx - self.window_radius
        # 入ってきた distances を対応する相対位置に挿入（短くてもOK）
        for i, val in enumerate(distances):
            abs_idx = start_abs + i
            rel = abs_idx - cl_idx
            rel_pos = rel + self.window_radius
            if 0 <= rel_pos < len(y):
                if val == '':
                    continue
                y[rel_pos] = val
    
        # Matplotlibで描画（中央0=cl_idxの位置を固定する）
        fig = plt.figure(figsize=fig_size, dpi=dpi)
        ax = fig.add_subplot(111)
        ax.grid(True)
        ax.set_xlabel("location (index)")
        ax.set_ylabel("d")
        ax.set_title(f"cl_idx={cl_idx} (sg_rel={sg_rel})")
    
        # 青の折れ線（点は一度すべて青丸にしてから、重ねてcl/sgを強調）
        ax.plot(x_rel, y, linestyle='-', marker='o', color='blue', markersize=5)
        for xr_idx, xa_idx in zip(x_rel, x_abs_labels):
            color = 'orange' if xa_idx % 2 == 0 else 'green'
            ax.axvline(xr_idx, color=color, linestyle='--', linewidth=2, alpha=0.7)
    
        # cl_idx（相対0）を黄色い丸で強調（値が存在する場合のみ）
        cl_pos = self.window_radius  # x_rel==0 の配列位置
        if not np.isnan(y[cl_pos]):
            ax.plot([0], [y[cl_pos]], marker='o', color='yellow', markersize=9, markeredgecolor='black')
    
        # sg（cl_idxからの相対：sg_rel）を赤丸で強調（窓内かつ値がある場合のみ）
        if -self.window_radius <= sg_rel <= self.window_radius:
            sg_pos = sg_rel + self.window_radius
            if not np.isnan(y[sg_pos]):
                ax.plot([sg_rel], [y[sg_pos]], marker='o', color='red', markersize=9, markeredgecolor='black')
    
        # x軸は相対位置で固定しつつ、目盛ラベルは絶対地点を表示
        ax.set_xlim(x_rel[0]-0.5, x_rel[-1]+0.5)
        ax.set_xticks(x_rel)
        ax.set_xticklabels([str(a) for a in x_abs_labels])

        if ylim is not None:
            ymin,ymax = ylim
            ax.set_ylim(ymin, ymax)
            ax.set_yticks(np.arange(ymin, ymax+1, 1))
    
        # goal_idxが与えられていれば、参考に縦線を入れてもよい（任意）
        if goal_idx is not None:
            # goalが窓内にあるときだけ相対位置に変換して破線を描画
            g_rel = goal_idx - cl_idx
            if -self.window_radius <= g_rel <= self.window_radius:
                ax.axvline(g_rel, linestyle='--')
    
        img = self._frame_to_bgr(fig)
        plt.close(fig)
        return img



    def load(self, path, start_time=None, goal_idx=None):
        with open(path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            raw_data_dict = {}

            gloabl_max_distance = 0 # グラフの縦軸用にdistanceの最大値を求める
            for row in reader:
                try:
                    time, sg_idx, cl_idx = row['time'], row['sg_idx'], row['cl_idx']
                    distances = []
                    for col_name in self.target_columns:
                        d_cvrt = np.nan if row[col_name] == '' else float(row[col_name])
                        distances.append(d_cvrt)
                    
                    gloabl_max_distance = max(np.nanmax(distances), gloabl_max_distance)
                    # ナノ秒単位の時刻からrospy.Timeへ変換
                    t_ns = int(time)
                    secs = t_ns // 10**9 # 秒とナノ秒に分ける
                    nsecs = t_ns % 10**9
                    time_cvrt = rospy.Time(secs, nsecs) # rospy.Timeオブジェクトに変換

                    raw_data_dict[time_cvrt] = {"distances":distances, "sg_idx":sg_idx, "cl_idx":cl_idx}
                except KeyError as e:
                    raise ValueError(f"CSVにtargetカラムまたは必須カラムがありません: {e}")
        
        # 外部からのstart_timeの指定が無い場合にはstart_timeを計算
        if start_time is None:
            start_times = [_msg_time_to_sec(time) for time in raw_data_dict.keys()]
            start_time = min(start_times)
        

        # graph_dictの作成
        ylim = (0, gloabl_max_distance)
        graph_dict = {}
        for t, data in raw_data_dict.items():
            curr_time = _msg_time_to_sec(t)
            # start_timeからの経過時間を計算し、frameに変換
            frame = int((curr_time - start_time) * self.loader_fps) # :check:
            
            # グラフを描画してcv2形式にする
            img = self._make_frame_image(data, ylim, goal_idx=goal_idx)
            graph_dict[frame] = img
        
        # このクラスのデータ想定では単独トラックにするので注意(len(track_metadatas)=1)
        track_records = {"distances": graph_dict}

        # start_frame及びend_frameを計算, ClipDataをtitleを"distances"にして作成
        start_frame = min(graph_dict.keys()) # なお、start_time変数はグローバルなminであるのに対して、これはlocal、つまりこのトピック内のみで最小
        end_frame = max(graph_dict.keys())
        duration_frames = end_frame - start_frame + 1
        clip_data = ClipData(title="distances", start_frame=start_frame, duration_frames=duration_frames)

        # TrackMetaDataをname=topic名で作成, add_clip
        track_metadata = TrackMetaData(name="distances", ui_height=60)
        track_metadata.add_clip(clip_data)
        track_metadatas = [track_metadata]

        return AnimeGraphData(track_metadatas=track_metadatas, track_records=track_records, fps=self.loader_fps, start_time=start_time)


def _msg_time_to_sec(t):
	# rospy.Time互換のオブジェクトをfloat秒へ
	# (t.secs, t.nsecs) を持つ前提
	return float(t.secs) + float(t.nsecs) * 1e-9

# pathからRecordDataを作成する
class RosbagLoader(DataLoader):
    def load(self, path:str, with_blur=False, start_time = None) -> RosbagData:

        # Bagファイルを読み込み
        bag = rosbag.Bag(path)
        topic_dict = {} # {'tpc0':[(time, msg), ...], 'tpc1':[...], ...}
        for info in bag.read_messages():
            topic, msg, t = info
            if not topic in topic_dict.keys():
                topic_dict[topic] = []
            topic_dict[topic].append((t, msg))
        bag.close()
        
        
        # 複数トピックにまたがるglobalなstart_timeを計算
        if start_time is None:
            start_times = [_msg_time_to_sec(v[0][0]) for v in topic_dict.values()]
            start_time = min(start_times)
        

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

            # start_frame及びend_frameを計算, ClipDataをtitleをトピック名して作成
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
        if with_blur:
            return RosbagWithBlurData(track_metadatas, track_records, self.loader_fps, start_time=start_time)
        else:
            return RosbagData(track_metadatas, track_records, self.loader_fps, start_time=start_time)
