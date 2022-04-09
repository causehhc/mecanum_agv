from lib.interface.camera import CameraInterface
import cv2
import numpy as np


class CamMap:
    def __init__(self):
        self.camera = CameraInterface("/sim/smallCar/camera/image_raw/compressed")
        self.frame = np.zeros((600, 600, 3), np.uint8)

    def update(self):
        self.frame = self.camera.frame
        return cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

