from designer.untitled import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys

from interface.remote import RemoteInterface
from interface.lidar import LidarInterface
from interface.camera import CameraInterface

import rospy


class MyMainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.remoteNode = RemoteInterface('/sim/smallCar/cmd_vel')
        self.LidarNode = LidarInterface('2')
        self.CameraNode = CameraInterface('3')

        self.pushButton.pressed.connect(self.wtf)

    def wtf(self):
        ret = self.remoteNode.move_cmd_send([0, 0, 0, 0])
        self.label.setText("{}".format(ret))


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
