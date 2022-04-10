from lib.interface.map import MapInterface
from lib.interface.path import PathInterface
from lib.interface.pose import PoseInterface
import cv2
import numpy as np


class BaseMap:
    def __init__(self):
        self.robot_radius = 5
        self.map = MapInterface("/map")
        self.pose = PoseInterface("/slam_out_pose")
        self.path = PathInterface(self.map.bitmap, self.robot_radius)
        self.frame = np.zeros((600, 600, 3), np.uint8)
        self.end = None
        self.layerList = []
        self.pathList = []
        self.procList = []

        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)


    def get_map(self):
        print('get_map')
        self.layerList = self.path.find_map()
        print('get_map ok')

    def get_path(self):
        print('get_path')
        if self.end is not None:
            self.pathList, self.procList = self.path.find_path((self.pose.x, self.pose.y), self.end)
        print('get_path ok')

    def get_clear(self):
        print('get_clear')
        self.procList = []
        self.pathList = []
        print('get_clear ok')

    def update_frame(self):
        # draw base_map
        self.frame = self.map.frame.copy()
        # self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

    def update_path(self):
        # draw proc
        for item in self.procList:
            x1 = int(item[0] - self.robot_radius / 2)
            y1 = int(item[1] - self.robot_radius / 2)
            x2 = int(item[0] + self.robot_radius / 2)
            y2 = int(item[1] + self.robot_radius / 2)
            cv2.rectangle(self.frame, (y1, x1), (y2, x2), (255, 255, 0), 1)  # sb

        # draw path
        for item in self.pathList:
            x1 = int(item[0] - self.robot_radius / 2)
            y1 = int(item[1] - self.robot_radius / 2)
            x2 = int(item[0] + self.robot_radius / 2)
            y2 = int(item[1] + self.robot_radius / 2)
            cv2.rectangle(self.frame, (y1, x1), (y2, x2), (255, 0, 0), 1)  # blue
        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

    def update_layer(self):
        # draw base_layer
        for item in self.layerList:
            self.frame[item[0]][item[1]] = (255, 0, 0)  # blue

        # draw self_pos
        x1 = int(self.pose.x - self.robot_radius / 2)
        y1 = int(self.pose.y - self.robot_radius / 2)
        x2 = int(self.pose.x + self.robot_radius / 2)
        y2 = int(self.pose.y + self.robot_radius / 2)
        cv2.rectangle(self.frame,(y1, x1), (y2, x2), (0,0,225), 1)# red

        # draw self_end
        if self.end is not None:
            x1 = int(self.end[0] - self.robot_radius / 2)
            y1 = int(self.end[1] - self.robot_radius / 2)
            x2 = int(self.end[0] + self.robot_radius / 2)
            y2 = int(self.end[1] + self.robot_radius / 2)
            cv2.rectangle(self.frame,(y1, x1), (y2, x2), (0,0,225), 1)# red

        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

