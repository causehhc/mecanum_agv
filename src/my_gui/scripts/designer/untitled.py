# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'untitled.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(914, 1021)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.groupBox_3)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_lidar = QtWidgets.QLabel(self.groupBox_3)
        self.label_lidar.setObjectName("label_lidar")
        self.verticalLayout_3.addWidget(self.label_lidar)
        self.label_cam = QtWidgets.QLabel(self.groupBox_3)
        self.label_cam.setObjectName("label_cam")
        self.verticalLayout_3.addWidget(self.label_cam)
        self.horizontalLayout_2.addWidget(self.groupBox_3)
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_map = QtWidgets.QLabel(self.groupBox_2)
        self.label_map.setObjectName("label_map")
        self.verticalLayout.addWidget(self.label_map)
        self.groupBox_5 = QtWidgets.QGroupBox(self.groupBox_2)
        self.groupBox_5.setObjectName("groupBox_5")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.groupBox_5)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton_1 = QtWidgets.QPushButton(self.groupBox_5)
        self.pushButton_1.setObjectName("pushButton_1")
        self.horizontalLayout.addWidget(self.pushButton_1)
        self.pushButton_2 = QtWidgets.QPushButton(self.groupBox_5)
        self.pushButton_2.setObjectName("pushButton_2")
        self.horizontalLayout.addWidget(self.pushButton_2)
        self.pushButton_3 = QtWidgets.QPushButton(self.groupBox_5)
        self.pushButton_3.setObjectName("pushButton_3")
        self.horizontalLayout.addWidget(self.pushButton_3)
        self.pushButton_4 = QtWidgets.QPushButton(self.groupBox_5)
        self.pushButton_4.setObjectName("pushButton_4")
        self.horizontalLayout.addWidget(self.pushButton_4)
        self.pushButton_5 = QtWidgets.QPushButton(self.groupBox_5)
        self.pushButton_5.setObjectName("pushButton_5")
        self.horizontalLayout.addWidget(self.pushButton_5)
        self.pushButton_6 = QtWidgets.QPushButton(self.groupBox_5)
        self.pushButton_6.setObjectName("pushButton_6")
        self.horizontalLayout.addWidget(self.pushButton_6)
        self.verticalLayout.addWidget(self.groupBox_5)
        self.horizontalLayout_2.addWidget(self.groupBox_2)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.groupBox_10 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_10.setObjectName("groupBox_10")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.groupBox_10)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.lineEdit_piip = QtWidgets.QLineEdit(self.groupBox_10)
        self.lineEdit_piip.setObjectName("lineEdit_piip")
        self.verticalLayout_6.addWidget(self.lineEdit_piip)
        self.verticalLayout_2.addWidget(self.groupBox_10)
        self.verticalLayout_9 = QtWidgets.QVBoxLayout()
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.checkBox_real = QtWidgets.QCheckBox(self.groupBox)
        self.checkBox_real.setObjectName("checkBox_real")
        self.verticalLayout_9.addWidget(self.checkBox_real)
        self.pushButton_connect = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_connect.setObjectName("pushButton_connect")
        self.verticalLayout_9.addWidget(self.pushButton_connect)
        self.pushButton_cancel = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_cancel.setObjectName("pushButton_cancel")
        self.verticalLayout_9.addWidget(self.pushButton_cancel)
        self.verticalLayout_2.addLayout(self.verticalLayout_9)
        self.groupBox_7 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_7.setObjectName("groupBox_7")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.groupBox_7)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_11 = QtWidgets.QLabel(self.groupBox_7)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_12.addWidget(self.label_11)
        self.label_sta_roscore = QtWidgets.QLabel(self.groupBox_7)
        self.label_sta_roscore.setObjectName("label_sta_roscore")
        self.horizontalLayout_12.addWidget(self.label_sta_roscore)
        self.verticalLayout_5.addLayout(self.horizontalLayout_12)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_9 = QtWidgets.QLabel(self.groupBox_7)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_11.addWidget(self.label_9)
        self.label_sta_gazebo = QtWidgets.QLabel(self.groupBox_7)
        self.label_sta_gazebo.setObjectName("label_sta_gazebo")
        self.horizontalLayout_11.addWidget(self.label_sta_gazebo)
        self.verticalLayout_5.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_7 = QtWidgets.QLabel(self.groupBox_7)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_10.addWidget(self.label_7)
        self.label_sta_slam = QtWidgets.QLabel(self.groupBox_7)
        self.label_sta_slam.setObjectName("label_sta_slam")
        self.horizontalLayout_10.addWidget(self.label_sta_slam)
        self.verticalLayout_5.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_5 = QtWidgets.QLabel(self.groupBox_7)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_9.addWidget(self.label_5)
        self.label_sta_astar = QtWidgets.QLabel(self.groupBox_7)
        self.label_sta_astar.setObjectName("label_sta_astar")
        self.horizontalLayout_9.addWidget(self.label_sta_astar)
        self.verticalLayout_5.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_3 = QtWidgets.QLabel(self.groupBox_7)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_8.addWidget(self.label_3)
        self.label_sta_bezier = QtWidgets.QLabel(self.groupBox_7)
        self.label_sta_bezier.setObjectName("label_sta_bezier")
        self.horizontalLayout_8.addWidget(self.label_sta_bezier)
        self.verticalLayout_5.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label = QtWidgets.QLabel(self.groupBox_7)
        self.label.setObjectName("label")
        self.horizontalLayout_7.addWidget(self.label)
        self.spinBox_sta_radius = QtWidgets.QSpinBox(self.groupBox_7)
        self.spinBox_sta_radius.setReadOnly(False)
        self.spinBox_sta_radius.setObjectName("spinBox_sta_radius")
        self.horizontalLayout_7.addWidget(self.spinBox_sta_radius)
        self.verticalLayout_5.addLayout(self.horizontalLayout_7)
        self.verticalLayout_2.addWidget(self.groupBox_7)
        self.groupBox_8 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_8.setObjectName("groupBox_8")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.groupBox_8)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.progressBar = QtWidgets.QProgressBar(self.groupBox_8)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.horizontalLayout_4.addWidget(self.progressBar)
        self.verticalLayout_2.addWidget(self.groupBox_8)
        self.groupBox_6 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_6.setObjectName("groupBox_6")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.groupBox_6)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.checkBox_remote = QtWidgets.QCheckBox(self.groupBox_6)
        self.checkBox_remote.setObjectName("checkBox_remote")
        self.verticalLayout_4.addWidget(self.checkBox_remote)
        self.pushButton_aopt = QtWidgets.QPushButton(self.groupBox_6)
        self.pushButton_aopt.setObjectName("pushButton_aopt")
        self.verticalLayout_4.addWidget(self.pushButton_aopt)
        self.pushButton_bopt = QtWidgets.QPushButton(self.groupBox_6)
        self.pushButton_bopt.setObjectName("pushButton_bopt")
        self.verticalLayout_4.addWidget(self.pushButton_bopt)
        self.pushButton_move = QtWidgets.QPushButton(self.groupBox_6)
        self.pushButton_move.setObjectName("pushButton_move")
        self.verticalLayout_4.addWidget(self.pushButton_move)
        self.verticalLayout_2.addWidget(self.groupBox_6)
        self.horizontalLayout_2.addWidget(self.groupBox)
        self.verticalLayout_8.addLayout(self.horizontalLayout_2)
        self.groupBox_9 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_9.setObjectName("groupBox_9")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.groupBox_9)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.textBrowser = QtWidgets.QTextBrowser(self.groupBox_9)
        self.textBrowser.setObjectName("textBrowser")
        self.verticalLayout_7.addWidget(self.textBrowser)
        self.verticalLayout_8.addWidget(self.groupBox_9)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 914, 23))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_3.setTitle(_translate("MainWindow", "REAL"))
        self.label_lidar.setText(_translate("MainWindow", "No Signal"))
        self.label_cam.setText(_translate("MainWindow", "No Signal"))
        self.groupBox_2.setTitle(_translate("MainWindow", "MAP"))
        self.label_map.setText(_translate("MainWindow", "No Signal"))
        self.groupBox_5.setTitle(_translate("MainWindow", "op"))
        self.pushButton_1.setText(_translate("MainWindow", "<"))
        self.pushButton_2.setText(_translate("MainWindow", "^"))
        self.pushButton_3.setText(_translate("MainWindow", "v"))
        self.pushButton_4.setText(_translate("MainWindow", ">"))
        self.pushButton_5.setText(_translate("MainWindow", "+"))
        self.pushButton_6.setText(_translate("MainWindow", "-"))
        self.groupBox.setTitle(_translate("MainWindow", "TRY THIS SHIT"))
        self.groupBox_10.setTitle(_translate("MainWindow", "PI_IP"))
        self.checkBox_real.setText(_translate("MainWindow", "RealCar"))
        self.pushButton_connect.setText(_translate("MainWindow", "Connect"))
        self.pushButton_cancel.setText(_translate("MainWindow", "Cancel"))
        self.groupBox_7.setTitle(_translate("MainWindow", "STATUS"))
        self.label_11.setText(_translate("MainWindow", "roscore:"))
        self.label_sta_roscore.setText(_translate("MainWindow", "offline"))
        self.label_9.setText(_translate("MainWindow", "gazebo:"))
        self.label_sta_gazebo.setText(_translate("MainWindow", "offline"))
        self.label_7.setText(_translate("MainWindow", "SLAM:"))
        self.label_sta_slam.setText(_translate("MainWindow", "offline"))
        self.label_5.setText(_translate("MainWindow", "Astar:"))
        self.label_sta_astar.setText(_translate("MainWindow", "offline"))
        self.label_3.setText(_translate("MainWindow", "Bezier:"))
        self.label_sta_bezier.setText(_translate("MainWindow", "offline"))
        self.label.setText(_translate("MainWindow", "Radius:"))
        self.groupBox_8.setTitle(_translate("MainWindow", "PROCESS"))
        self.groupBox_6.setTitle(_translate("MainWindow", "TRY_THIS_SHIT"))
        self.checkBox_remote.setText(_translate("MainWindow", "ArtificialRemote"))
        self.pushButton_aopt.setText(_translate("MainWindow", "Astar-opt"))
        self.pushButton_bopt.setText(_translate("MainWindow", "Bezier-opt"))
        self.pushButton_move.setText(_translate("MainWindow", "Move"))
        self.groupBox_9.setTitle(_translate("MainWindow", "LOG"))

