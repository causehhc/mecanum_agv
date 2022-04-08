from lib.interface.lidar import LidarInterface
import cv2
import numpy as np


class LidarMap:
    def __init__(self):
        self.lidar = LidarInterface("/sim/smallCar/laser/scan")
        self.frame = np.zeros((600, 600, 3), np.uint8)

    def update(self):
        self.frame = self.lidar.frame
        return cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

