import rospy
from sensor_msgs.msg import LaserScan

import cv2
import numpy as np
import math


class LidarInterface:
    def __init__(self, topic_name, size):
        self.size = size
        self.lidar = rospy.Subscriber(topic_name, LaserScan, self.callback, queue_size=1)
        self.frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)

    def callback(self, data):
        frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)
        angle = data.angle_min
        for r in data.ranges:
            if math.isinf(r):
                r = 0
            x = math.trunc((r * self.size[0]*0.1) * math.cos(angle + (180.0 * 3.1416 / 180.0)))
            y = math.trunc((r * self.size[0]*0.1) * math.sin(angle + (180.0 * 3.1416 / 180.0)))

            if y > self.size[1] or y < -self.size[1] or x < -self.size[0] or x > self.size[0]:
                x = 0
                y = 0

            cv2.line(frame, (
                int(self.size[0]/2),
                int(self.size[1]/2)
            ), (
                int(y + self.size[1]/2),
                int(x + self.size[0]/2)
            ), (255, 0, 0), 2)
            angle = angle + data.angle_increment
            cv2.circle(frame, (int(self.size[0]/2), int(self.size[0]/2)), 2, (255, 255, 0))
        self.frame = frame


def main():
    rospy.init_node('lidar_listener')
    lidar = LidarInterface("/scan", (300, 300))
    while True:
        frame = lidar.frame
        if frame is not None:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
