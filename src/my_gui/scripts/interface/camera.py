import rospy
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np


class CameraInterface:
    def __init__(self, topic_name):
        self.cam = rospy.Subscriber(topic_name, CompressedImage, self.callback, queue_size=1)
        self.frame = np.zeros((600, 600, 3), np.uint8)

    def callback(self, data):
        frame = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        origin_size = frame.shape
        aim_size = 600
        param = aim_size / origin_size[1]
        now_size = (int(origin_size[1] * param), int(origin_size[0] * param))
        self.frame = cv2.resize(frame, now_size)

    def get_frame_qt(self):
        return cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)


def main():
    rospy.init_node('cam_listener')
    cam = CameraInterface("/sim/smallCar/camera/image_raw/compressed")
    while True:
        frame = cam.frame
        if frame is not None:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
