from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap

from designer.untitled import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt
import sys

from interface.remote import RemoteInterface
from interface.lidar import LidarInterface
from interface.camera import CameraInterface
from interface.map import MapInterface
from interface.path import PathInterface

from algorithm.plan.astar import astar

import rospy


class MyMainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.camera = CameraInterface("/sim/smallCar/camera/image_raw/compressed")
        self.lidar = LidarInterface("/sim/smallCar/laser/scan")
        self.map = MapInterface("/map", "/slam_out_pose")
        self.path = PathInterface()

        self.label_cam.setScaledContents(False)
        self.label_lidar.setScaledContents(False)
        self.label_map.setScaledContents(False)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.play)
        self._timer.start(100)

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            label_x = self.label_map.x()
            label_y = self.label_map.y()
            mouse_x = event.x()
            mouse_y = event.y()
            # real_x = mouse_x - label_x
            # real_y = mouse_y - label_y
            real_x = mouse_y - label_y
            real_y = mouse_x - label_x

            if 0 <= real_x <= 600 and 0 <= real_y <= 600:
                # self.path.find_path(self.map.frame, self.map.pose_pos, (real_x, real_y))
                # self.path.creat_path_img()
                # TODO
                print(self.map.pose_pos, (real_x, real_y))
            event.accept()

    def play(self):
        def convertFrame(frame):
            height, width = frame.shape[:2]
            frame = QImage(frame, width, height, QImage.Format_RGB888)
            frame = QPixmap.fromImage(frame)
            return frame

        try:
            self.label_cam.setPixmap(convertFrame(self.camera.get_frame_qt()))
            self.label_lidar.setPixmap(convertFrame(self.lidar.get_frame_qt()))
            self.label_map.setPixmap(convertFrame(self.map.get_frame_qt()))
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
