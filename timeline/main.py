# main.py

import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from view.widgets import TimelineWidget
from model.data import TrackData, ClipData

def main():
    app = QApplication(sys.argv)
    main_window = QMainWindow()
    main_window.setWindowTitle("Editorial Timeline")

    # テーマ名か辞書を渡せるようにしました
    timelineWidget = TimelineWidget(theme="dark")
    main_window.setCentralWidget(timelineWidget)

    # --- スクリプトによるトラック／クリップ追加対応 ---
    # TrackData は名前と表示高さ(height)を受け取るように拡張しています
    trackv2 = TrackData(name="Video 2", height=30)
    trackv1 = TrackData(name="Video 1", height=30)
    tracka1 = TrackData(name="Audio 1", height=30)
    tracka2 = TrackData(name="Audio 2", height=30)

    # ClipData はタイトル・開始フレーム・継続フレーム数を受け取るように変更
    trackv1.add_clip(ClipData(title="Clip_A", start_frame=10, duration_frames=50))
    trackv1.add_clip(ClipData(title="Clip_B", start_frame=70, duration_frames=40))

    tracka2.add_clip(ClipData(title="Sound_A", start_frame=20, duration_frames=60))
    tracka2.add_clip(ClipData(title="Music_08", start_frame=90, duration_frames=30))

    trackv2.add_clip(ClipData(title="Mov_A", start_frame=0,  duration_frames=60))
    trackv2.add_clip(ClipData(title="Avi_B", start_frame=61, duration_frames=90))

    tracka1.add_clip(ClipData(title="Music_16", start_frame=0,  duration_frames=30))
    tracka1.add_clip(ClipData(title="Sound_B", start_frame=120,duration_frames=30))

    # TimelineWidget 側に add_track / update_layout を実装済み
    timelineWidget.add_track(trackv2)
    timelineWidget.add_track(trackv1)
    timelineWidget.add_track(tracka1)
    timelineWidget.add_track(tracka2)

    # このメソッドですべてのアイテムをシーンに再配置します
    timelineWidget.update_layout()

    main_window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
