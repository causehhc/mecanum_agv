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
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_map = QtWidgets.QLabel(self.centralwidget)
        self.label_map.setObjectName("label_map")
        self.verticalLayout.addWidget(self.label_map)
        self.label_lidar = QtWidgets.QLabel(self.centralwidget)
        self.label_lidar.setObjectName("label_lidar")
        self.verticalLayout.addWidget(self.label_lidar)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.label_cam = QtWidgets.QLabel(self.centralwidget)
        self.label_cam.setObjectName("label_cam")
        self.horizontalLayout.addWidget(self.label_cam)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 23))
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
        self.label_map.setText(_translate("MainWindow", "TextLabel"))
        self.label_lidar.setText(_translate("MainWindow", "TextLabel"))
        self.label_cam.setText(_translate("MainWindow", "TextLabel"))

