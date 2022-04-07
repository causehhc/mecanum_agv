import rospy
from sensor_msgs.msg import LaserScan

import cv2
import numpy as np
import math


class LidarInterface:
    def __init__(self, topic_name):
        self.lidar = rospy.Subscriber(topic_name, LaserScan, self.callback, queue_size=1)
        self.frame = np.zeros((600, 600, 3), np.uint8)

    def callback(self, data):
        frame = np.zeros((600, 600, 3), np.uint8)
        angle = data.angle_min
        for r in data.ranges:
            if math.isinf(r):
                r = 0
            x = math.trunc((r * 50.0) * math.cos(angle + (90.0 * 3.1416 / 180.0)))
            y = math.trunc((r * 50.0) * math.sin(angle + (90.0 * 3.1416 / 180.0)))

            if y > 600 or y < -600 or x < -600 or x > 600:
                x = 0
                y = 0

            cv2.line(frame, (300, 300), (x + 300, y + 300), (255, 0, 0), 2)
            angle = angle + data.angle_increment
            cv2.circle(frame, (300, 300), 2, (255, 255, 0))
        self.frame = frame


def main():
    rospy.init_node('lidar_listener')
    lidar = LidarInterface("/sim/smallCar/laser/scan")
    while True:
        frame = lidar.frame
        if frame is not None:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
