# main.py

import sys
from PySide6.QtWidgets import QApplication, QMainWindow
from view.widgets import TimelineWidget
from model.data import TrackMetaData, ClipData, MockData

def main():
    app = QApplication(sys.argv)
    main_window = QMainWindow()
    main_window.setWindowTitle("Editorial Timeline")
    
    # Trackdataを作成 => ClipDataをadd => MockDataを作成 => self.controller.add_recordで追加の流れ
    # もしくはloaderによりMockDataを作成 => self.controller.add_recordで追加の流れ
    # loaderを使わない場合、controllerのfpsを利用してRecordDataを作成


    # テーマ名か辞書を渡せるようにしました
    timelineWidget = TimelineWidget(theme="dark")
    main_window.setCentralWidget(timelineWidget)

    # --- スクリプトによるトラック／クリップ追加対応 ---
    # TrackMetaData は名前と表示高さ(ui_height)を受け取るように拡張しています
    trackv2 = TrackMetaData(name="Video 2", ui_height=60)
    trackv1 = TrackMetaData(name="Video 1", ui_height=60)
    tracka1 = TrackMetaData(name="Audio 1", ui_height=60)
    tracka2 = TrackMetaData(name="Audio 2", ui_height=60)

    # ClipData はタイトル・開始フレーム・継続フレーム数を受け取るように変更
    trackv1.add_clip(ClipData(title="Clip_A", start_frame=10, duration_frames=50))
    trackv1.add_clip(ClipData(title="Clip_B", start_frame=70, duration_frames=40))

    tracka2.add_clip(ClipData(title="Sound_A", start_frame=20, duration_frames=60))
    tracka2.add_clip(ClipData(title="Music_08", start_frame=90, duration_frames=30))

    trackv2.add_clip(ClipData(title="Mov_A", start_frame=0,  duration_frames=60))
    trackv2.add_clip(ClipData(title="Avi_B", start_frame=61, duration_frames=90))

    tracka1.add_clip(ClipData(title="Music_16", start_frame=0,  duration_frames=30))
    tracka1.add_clip(ClipData(title="Sound_B", start_frame=120,duration_frames=30))

    # # TimelineWidget 側に add_track / update_layout を実装済み
    # timelineWidget.controller.add_record(trackv2)
    # timelineWidget.controller.add_record(trackv1)
    # timelineWidget.controller.add_record(tracka1)
    # timelineWidget.controller.add_record(tracka2)

    # MockData インスタンスの作成
    mock_data = MockData(track_metadatas=[trackv1, trackv2, tracka1, tracka2], fps=24)

    timelineWidget.controller.add_record(record_data=mock_data)

    # # このメソッドですべてのアイテムをシーンに再配置します
    # timelineWidget.update_layout()

    main_window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
