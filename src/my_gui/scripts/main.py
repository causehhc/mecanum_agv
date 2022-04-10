from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap

from designer.untitled import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
import sys

from lib.map.BaseMap import BaseMap
from lib.map.camMap import CamMap
from lib.map.lidarMap import LidarMap

import rospy
import _thread
from multiprocessing import Process


class MyMainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

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
        _thread.start_new_thread(self.test1_1, (1,))
        _thread.start_new_thread(self.test1_2, (1,))
        _thread.start_new_thread(self.test1_3, (1,))
        _thread.start_new_thread(self.test1_4, (1,))
        _thread.start_new_thread(self.test1_5, (1,))

    def pushButton_2_func(self):
        _thread.start_new_thread(self.test2_1, (1,))


    def pushButton_3_func(self):
        _thread.start_new_thread(self.test2_2, (1,))

    def pushButton_4_func(self):
        _thread.start_new_thread(self.test2_3, (1,))

    def test1_1(self, haha=1):
        while True:
            self.camView.update()

    def test1_2(self, haha=1):
        while True:
            self.lidarView.update()

    def test1_3(self, haha=1):
        while True:
            self.mapView.update_layer()

    def test1_4(self, haha=1):
        while True:
            self.mapView.update_path()

    def test1_5(self, haha=1):
        while True:
            self.mapView.update_frame()

    def test2_1(self, haha=1):
        self.mapView.get_map()
    def test2_2(self, haha=1):
        self.mapView.get_path()
    def test2_3(self, haha=1):
        self.mapView.get_clear()

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            real_x = event.x() - self.label_map.x()
            real_y = event.y() - self.label_map.y()
            if 0 <= real_x <= 600 and 0 <= real_y <= 600:
                self.mapView.end = (real_y, real_x)
            else:
                self.mapView.end = None
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
        self.label_cam.setPixmap(convertFrame(self.camView.returnFrame))
        self.label_lidar.setPixmap(convertFrame(self.lidarView.returnFrame))
        self.label_map.setPixmap(convertFrame(self.mapView.returnFrame))


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
