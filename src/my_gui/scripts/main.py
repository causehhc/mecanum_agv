import os
import time

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
from algorithm.slam.hector import Hetor_SLAM

from lib.submodule.tcp_client import Client
from lib.submodule.ipaddr import *

import rospy
import _thread
# from multiprocessing import Process
import subprocess


class MyMainWindow(QMainWindow, Ui_MainWindow):
    text_update = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.robot_radius = 4

        # print
        self.text_update.connect(self.append_text)
        sys.stdout = self

        self.client = None
        self.my_ip = get_host_ip()
        self.pi_ip = "192.168.2.184"
        self.p_core = None
        self.sm = Hetor_SLAM()
        self.lineEdit_piip.setText(self.pi_ip)

        self.remote = RemoteInterface("/cmd_vel")
        self.camView = CamMap()
        self.lidarView = LidarMap()
        self.mapView = BaseMap()

        self.has_start_main = None
        self.sta_remote = None
        self.sta_work_path = None
        self.sta_work_aopt = None
        self.sta_work_bopt = None
        self.sta_work_move = None
        self.pushButton_init()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.thread_flashGUI_display)
        self._timer.start(30)

    def write(self, text):
        self.text_update.emit(str(text))

    def flush(self):
        pass

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

    def print_log(self, data):
        print(data)

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            real_x = event.x() - self.groupBox_2.x()
            real_y = event.y() - self.groupBox_2.y()
            real_x -= 15
            real_y -= 40
            if 0 <= real_x <= 600 and 0 <= real_y <= 600:
                self.mapView.end = (real_y, real_x)
                self.sta_work_path = True
                # print(self.mapView.pose.x, self.mapView.pose.y, real_y, real_x)
            else:
                self.mapView.end = None
                self.sta_work_path = False
            event.accept()
        elif event.buttons() == Qt.RightButton:
            self.mapView.end = None
            self.sta_work_path = False
            event.accept()

    def closeEvent(self, event):
        self.cancel_car()

    def connect_car(self):
        if self.has_start_main is None:
            if self.checkBox_real.checkState():
                self.client = Client()
                self.pi_ip = self.lineEdit_piip.text()
                self.client.connect(self.pi_ip, 15057)

                self.client.send_msg("IP {}".format(self.pi_ip))
                recv = self.client.recv_msg()
                if recv != 'ok':
                    self.print_log("set pi ROS_IP: False")
                    return False
                else:
                    self.print_log("set pi ROS_IP: {}".format(self.pi_ip))

                self.client.send_msg("MASTER http://{}:11311/".format(self.my_ip))
                recv = self.client.recv_msg()
                if recv != 'ok':
                    self.print_log("set pi ROS_MASTER_URI: False")
                    return False
                else:
                    self.print_log("set pi ROS_MASTER_URI: ...{}...".format(self.my_ip))

                # os.environ["ROS_IP"] = self.my_ip
                # self.print_log("set ubuntu ROS_IP: {}".format(self.my_ip))
                # os.environ["ROS_MASTER_URI"] = "http://{}:11311/".format(self.my_ip)
                # self.print_log("set ubuntu ROS_MASTER_URI: ...{}...".format(self.my_ip))

                # self.p_core = subprocess.Popen("roscore", shell=True)
                # time.sleep(10)

                self.client.send_msg("start")
                recv = self.client.recv_msg()
                if recv != 'ok':
                    self.print_log("start pi driver False")
                    return False
                else:
                    self.print_log("start pi driver")

            rospy.init_node('my_gui')
            rospy.on_shutdown(shutdown)
            self.print_log("Read to Go!!!")

            self.sm.config()
            self.sm.start()
            _thread.start_new_thread(self.thread_work, (1,))
            _thread.start_new_thread(self.thread_remote, (1,))
            _thread.start_new_thread(self.thread_flashGUI_info, (1,))
            _thread.start_new_thread(self.thread_flashStatus, (1,))
            self.has_start_main = True
        else:
            self.print_log('Has connected')

    def cancel_car(self):
        if self.client:
            self.sm.stop()
            self.client.send_msg("stop")
            self.client.recv_msg()
            self.client.send_msg("exit")
            # self.p_core.kill()
            self.client = None

    def pushButton_init(self):
        # def pushButton_():
        #     pass
        # self.pushButton_.clicked.connect(pushButton_)

        def pushButton_connect():
            self.connect_car()

        def pushButton_cancel():
            self.cancel_car()

        def checkBox_remote():
            sta = self.checkBox_remote.checkState()
            if sta:
                self.sta_remote = True
            else:
                self.sta_remote = False

        def pushButton_aopt():
            self.sta_work_aopt = True

        def pushButton_bopt():
            self.sta_work_bopt = True

        def pushButton_move():
            self.sta_work_move = True
            self.sta_remote = False
            self.checkBox_remote.setCheckState(False)

        def spinBox_sta_radius():
            val = self.spinBox_sta_radius.value()
            self.mapView.view_robot_radius += (val - self.robot_radius)
            self.robot_radius = val
            self.mapView.clear_map()

        def pushButton_1():
            self.mapView.map.change_param(0, [-10, 0])
            self.mapView.pose.change_param(0, [-10, 0])
            self.mapView.map.image_transform()
            self.mapView.clear_map()

        def pushButton_2():
            self.mapView.map.change_param(0, [0, -10])
            self.mapView.pose.change_param(0, [0, -10])
            self.mapView.map.image_transform()
            self.mapView.clear_map()

        def pushButton_3():
            self.mapView.map.change_param(0, [0, 10])
            self.mapView.pose.change_param(0, [0, 10])
            self.mapView.map.image_transform()
            self.mapView.clear_map()

        def pushButton_4():
            self.mapView.map.change_param(0, [10, 0])
            self.mapView.pose.change_param(0, [10, 0])
            self.mapView.map.image_transform()
            self.mapView.clear_map()

        def pushButton_5():
            self.mapView.map.change_param(1, [0, 0])
            self.mapView.pose.change_param(1, [0, 0])
            self.mapView.view_robot_radius += 1
            self.mapView.map.image_transform()
            self.mapView.clear_map()

        def pushButton_6():
            self.mapView.map.change_param(-1, [0, 0])
            self.mapView.pose.change_param(-1, [0, 0])
            if self.mapView.view_robot_radius > 1:
                self.mapView.view_robot_radius -= 1
            self.mapView.map.image_transform()
            self.mapView.clear_map()

        self.pushButton_connect.clicked.connect(pushButton_connect)
        self.pushButton_cancel.clicked.connect(pushButton_cancel)
        self.checkBox_remote.clicked.connect(checkBox_remote)
        self.pushButton_aopt.clicked.connect(pushButton_aopt)
        self.pushButton_bopt.clicked.connect(pushButton_bopt)
        self.pushButton_move.clicked.connect(pushButton_move)
        self.spinBox_sta_radius.setValue(5)
        self.spinBox_sta_radius.valueChanged.connect(spinBox_sta_radius)
        self.pushButton_1.clicked.connect(pushButton_1)
        self.pushButton_2.clicked.connect(pushButton_2)
        self.pushButton_3.clicked.connect(pushButton_3)
        self.pushButton_4.clicked.connect(pushButton_4)
        self.pushButton_5.clicked.connect(pushButton_5)
        self.pushButton_6.clicked.connect(pushButton_6)

    def thread_work(self, haha=1):
        rate = rospy.Rate(50)  # 50ms
        while True:
            if self.sta_work_path is True:
                self.mapView.get_path()
                self.sta_work_path = False

            if self.sta_work_aopt is True:
                self.mapView.get_map(self.robot_radius)
                self.sta_work_aopt = False

            if self.sta_work_bopt is True:
                self.mapView.get_bezier()
                self.sta_work_bopt = False

            if self.sta_work_move is True:
                def setButton(a):
                    self.pushButton_1.setDisabled(a)
                    self.pushButton_2.setDisabled(a)
                    self.pushButton_3.setDisabled(a)
                    self.pushButton_4.setDisabled(a)
                    self.pushButton_5.setDisabled(a)
                    self.pushButton_6.setDisabled(a)

                setButton(True)
                self.mapView.get_move(self.remote, self.robot_radius)
                self.sta_work_move = False
                setButton(False)
            rate.sleep()

    def thread_flashStatus(self, haha=1):
        rate = rospy.Rate(10)  # 1000ms
        while True:
            if True:
                self.label_sta_roscore.setText('online')
                self.label_sta_gazebo.setText('online')
                self.label_sta_slam.setText('online')
                self.label_sta_astar.setText('online')
                self.label_sta_bezier.setText('online')
            rate.sleep()

    def thread_remote(self, haha=1):
        rate = rospy.Rate(50)  # 50ms
        while True:
            if self.sta_remote is True:
                self.remote.run_once()
            rate.sleep()

    def thread_flashGUI_info(self, haha=1):
        rate = rospy.Rate(20)  # 50ms
        while True:
            self.camView.update()
            self.lidarView.update()
            self.mapView.update()
            rate.sleep()

    def thread_flashGUI_display(self, haha=1):
        def convertFrame(frame):
            height, width = frame.shape[:2]
            frame = QImage(frame, width, height, QImage.Format_RGB888)
            frame = QPixmap.fromImage(frame)
            return frame

        self.label_cam.setPixmap(convertFrame(self.camView.returnFrame))
        self.label_lidar.setPixmap(convertFrame(self.lidarView.returnFrame))
        self.label_map.setPixmap(convertFrame(self.mapView.returnFrame))
        val = 100
        if len(self.mapView.pathList) != 0:
            val = int(len(self.mapView.pathList) * (100 / self.mapView.pathLen))
        self.progressBar.setValue(100 - val)


def shutdown():
    print("shutting down the pyqtNode...")


def main():
    app = QApplication(sys.argv)
    main_window = MyMainWindow()
    main_window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
