import math
import time

from lib.interface.map import MapInterface
from lib.interface.path import PathInterface
from lib.interface.pose import PoseInterface
import cv2
import numpy as np


class BaseMap:
    def __init__(self):
        self.size = (600, 600)
        self.scale = 5
        self.diff = [0, 0]
        self.view_robot_radius = 4
        self.map = MapInterface("/map", self.size, self.scale, self.diff)
        self.pose = PoseInterface("/slam_out_pose", self.size, self.scale, self.diff)
        self.path = PathInterface()
        self.frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)
        self.end = None
        self.layerList = []
        self.pathList = []
        self.pathLen = None
        self.procList = []
        self.bezierList = []
        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

    def get_map(self,r):
        self.layerList = self.path.find_map(self.map.bitmap, 1, self.view_robot_radius, self.scale,r)

    def clear_map(self):
        self.get_clear()
        self.path.runner = None
        self.layerList = []
        self.end = None

    def get_path(self):
        if self.end is not None:
            self.pathList, self.procList = self.path.find_path((self.pose.x, self.pose.y), self.end)
            self.pathLen = len(self.pathList)
            self.bezierList = []

    def get_bezier(self):
        if len(self.pathList) != 0:
            self.bezierList = self.path.find_bezier(self.pathList)

    def get_clear(self):
        self.procList = []
        self.pathList = []
        self.bezierList = []

    def get_move(self, remote, r):
        if len(self.bezierList) != 0:
            temp_list = list(self.bezierList)
            self.path.find_aim(temp_list, self.pose, remote, r)
        elif len(self.pathList) != 0:
            self.path.find_aim(self.pathList, self.pose, remote, r)
        elif self.end is not None:
            self.path.find_aim([self.end], self.pose, remote, r)

    def update_frame(self):
        # draw frame
        self.frame = self.map.frame.copy()

    def update_layer(self):
        # draw base_layer
        for item in self.layerList:
            self.frame[item[0]][item[1]] = (255, 0, 0)  # blue

        # draw self_pos
        x1 = int(self.pose.x - self.view_robot_radius / 2)
        y1 = int(self.pose.y - self.view_robot_radius / 2)
        x2 = int(self.pose.x + self.view_robot_radius / 2)
        y2 = int(self.pose.y + self.view_robot_radius / 2)
        cv2.rectangle(self.frame, (y1, x1), (y2, x2), (0, 0, 225), -1)  # red
        x3 = int(self.view_robot_radius * math.cos(self.pose.yaw) + self.pose.x)
        y3 = int(self.view_robot_radius * math.sin(self.pose.yaw) + self.pose.y)
        cv2.line(self.frame, (self.pose.y, self.pose.x), (y3, x3), (255, 0, 0), 1)

        # draw self_end
        if self.end is not None:
            x1 = int(self.end[0] - self.view_robot_radius / 2)
            y1 = int(self.end[1] - self.view_robot_radius / 2)
            x2 = int(self.end[0] + self.view_robot_radius / 2)
            y2 = int(self.end[1] + self.view_robot_radius / 2)
            cv2.rectangle(self.frame, (y1, x1), (y2, x2), (0, 0, 225), 1)  # red

    def update_path(self):
        if self.end is None:
            self.get_clear()

        # draw proc
        for item in self.procList:
            x1 = int(item[0] - self.view_robot_radius / 2)
            y1 = int(item[1] - self.view_robot_radius / 2)
            x2 = int(item[0] + self.view_robot_radius / 2)
            y2 = int(item[1] + self.view_robot_radius / 2)
            cv2.rectangle(self.frame, (y1, x1), (y2, x2), (255, 255, 0), 1)  # hahaBule

        # draw path
        for item in self.pathList:
            x1 = int(item[0] - self.view_robot_radius / 2)
            y1 = int(item[1] - self.view_robot_radius / 2)
            x2 = int(item[0] + self.view_robot_radius / 2)
            y2 = int(item[1] + self.view_robot_radius / 2)
            cv2.rectangle(self.frame, (y1, x1), (y2, x2), (255, 0, 0), 1)  # blue

        # draw bezier
        for item in self.bezierList:
            self.frame[item[0]][item[1]] = (0, 0, 255)  # red

    def update(self):
        self.update_frame()
        self.update_path()
        self.update_layer()
        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
