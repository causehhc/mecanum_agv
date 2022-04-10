from lib.interface.lidar import LidarInterface
import cv2
import numpy as np


class LidarMap:
    def __init__(self):
        self.size = (300, 300)
        self.lidar = LidarInterface("/scan", self.size)
        self.frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)
        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

    def update(self):
        # print(self.__str__())
        self.frame = self.lidar.frame
        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

