# main.py

import sys
from PySide6.QtWidgets import QApplication
from view.widgets import TimelineView

def main():
    # QApplication のインスタンスを作成
    app = QApplication(sys.argv)
    
    # メインウィンドウ（TimelineView）を生成・表示
    window = TimelineView()
    window.setWindowTitle("Timeline Editor")
    window.resize(800, 600)
    window.show()
    
    # イベントループ開始
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
