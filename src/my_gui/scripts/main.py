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
        self.end = None

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.play)
        self._timer.start(100)

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            real_x = event.x() - self.label_map.x()
            real_y = event.y() - self.label_map.y()
            if 0 <= real_x <= 600 and 0 <= real_y <= 600:
                self.end = (real_y, real_x)
                # print((self.mapView.pose.x, self.mapView.pose.y), (real_x, real_y))
                # print(self.mapView.map.bitmap[self.mapView.pose.x][self.mapView.pose.y], self.mapView.map.bitmap[real_x][real_y])
            else:
                self.end = None
            event.accept()
        elif event.buttons() == Qt.RightButton:
            self.end = None
            event.accept()

    def play(self):
        def convertFrame(frame):
            height, width = frame.shape[:2]
            frame = QImage(frame, width, height, QImage.Format_RGB888)
            frame = QPixmap.fromImage(frame)
            return frame
        try:
            self.label_cam.setPixmap(convertFrame(self.camView.update()))
            self.label_lidar.setPixmap(convertFrame(self.lidarView.update()))
            self.label_map.setPixmap(convertFrame(self.mapView.update(self.end)))
        except TypeError:
            print('No Frame')


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
