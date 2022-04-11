from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap

from designer.untitled import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QTextCursor
import sys

from lib.map.BaseMap import BaseMap
from lib.map.camMap import CamMap
from lib.map.lidarMap import LidarMap
from lib.interface.remote import RemoteInterface

import rospy
import _thread
from multiprocessing import Process
import time


class MyMainWindow(QMainWindow, Ui_MainWindow):
    text_update = pyqtSignal(str)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        # print
        self.text_update.connect(self.append_text)
        sys.stdout = self

        self.remote = RemoteInterface("/cmd_vel")
        self.camView = CamMap()
        self.lidarView = LidarMap()
        self.mapView = BaseMap()

        self.label_cam.setScaledContents(False)
        self.label_lidar.setScaledContents(False)
        self.label_map.setScaledContents(False)

        self.pushButton_1.clicked.connect(self.pushButton_1_func)
        self.pushButton_2.clicked.connect(self.pushButton_2_func)
        self.pushButton_3.clicked.connect(self.pushButton_3_func)
        self.pushButton_4.clicked.connect(self.pushButton_4_func)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.play)
        self._timer.start(100)

    def pushButton_1_func(self):
        _thread.start_new_thread(self.remote.run, (1,))

    def pushButton_2_func(self):
        _thread.start_new_thread(self.mapView.get_map, (1,))

    def pushButton_3_func(self):
        _thread.start_new_thread(self.mapView.get_path, (1,))

    def pushButton_4_func(self):
        _thread.start_new_thread(self.mapView.get_move, (self.remote, 1, ))

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            real_x = event.x() - self.groupBox_2.x()
            real_y = event.y() - self.groupBox_2.y()
            real_x -= 15
            real_y -= 40
            if 0 <= real_x <= 600 and 0 <= real_y <= 600:
                self.mapView.end = (real_y, real_x)
            else:
                self.mapView.end = None
            print(self.mapView.pose.x, self.mapView.pose.y, self.mapView.end)
            event.accept()
        elif event.buttons() == Qt.RightButton:
            self.mapView.end = None
            event.accept()

    def play(self):
        def convertFrame(frame):
            height, width = frame.shape[:2]
            frame = QImage(frame, width, height, QImage.Format_RGB888)
            frame = QPixmap.fromImage(frame)
            return frame

        self.camView.update()
        self.lidarView.update()
        self.mapView.update()
        self.label_cam.setPixmap(convertFrame(self.camView.returnFrame))
        self.label_lidar.setPixmap(convertFrame(self.lidarView.returnFrame))
        self.label_map.setPixmap(convertFrame(self.mapView.returnFrame))

    def write(self, text):
        self.text_update.emit(str(text))

    def flush(self):
        pass

    # ------------------------------
    # 5.槽函数
    def append_text(self, text):
        cur = self.textBrowser.textCursor()  # Move cursor to end of text
        cur.movePosition(QTextCursor.End)
        s = str(text)
        while s:
            head, sep, s = s.partition("\n")  # Split line at LF
            cur.insertText(head)  # Insert text at cursor
            if sep:  # New line if LF
                cur.insertBlock()
        self.textBrowser.setTextCursor(cur)  # Update visible cursor


def shutdown():
    print("shutting down the pyqtNode...")


def main():
    rospy.init_node('pyqtNode')
    rospy.on_shutdown(shutdown)

    app = QApplication(sys.argv)
    main_window = MyMainWindow()
    main_window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
