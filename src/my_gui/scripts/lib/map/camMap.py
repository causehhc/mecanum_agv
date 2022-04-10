from lib.interface.camera import CameraInterface
import cv2
import numpy as np


class CamMap:
    def __init__(self):
        self.size = (300, 300)
        self.camera = CameraInterface("/camera/image_raw/compressed", self.size)
        self.frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)
        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

    def update(self):
        # print(self.__str__())
        self.frame = self.camera.frame
        self.returnFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

