import rospy
from nav_msgs.msg import OccupancyGrid

import cv2
import numpy as np


class MapInterface:
    def __init__(self, topic_name):
        self.map = rospy.Subscriber(topic_name, OccupancyGrid, self.map_callback, queue_size=1)
        self.SCALE = 5  # max_resolution: 3.41333
        self.bitmap = np.zeros((600, 600, 1), np.uint8)
        self.frame = np.zeros((600, 600, 3), np.uint8)

    def image_scaling(self, data, scale):
        origin_length = data.info.height
        display_length = 600
        center_point = origin_length / 2

        length_start = int(center_point - (origin_length / scale) / 2)
        length_incremental = int(origin_length / scale)

        if scale < 3.413:
            for row in range(0, length_incremental):
                for col in range(0, length_incremental):
                    if data.data[(length_start + row)+(length_start + col) * origin_length] == -1:
                        tmp1 = 0  # UNKNOWN
                        tmp2 = (166, 166, 166)
                    elif data.data[(length_start + row)+(length_start + col) * origin_length] == 0:
                        tmp1 = 1  # PATH
                        tmp2 = (255, 255, 255)
                    else:
                        tmp1 = 2  # WALL
                        tmp2 = (0, 0, 0)
                    param = length_incremental / display_length
                    self.bitmap[int(row / param)][int(col / param)] = tmp1
                    self.frame[int(row / param)][int(col / param)] = tmp2
        else:
            for row in range(0, display_length):
                for col in range(0, display_length):
                    o_x = int(row / (display_length / length_incremental) + length_start)
                    o_y = int(col / (display_length / length_incremental) + length_start)
                    if data.data[o_x+o_y*origin_length] == -1:
                        tmp1 = 0
                        tmp2 = (166, 166, 166)
                    elif data.data[o_x+o_y*origin_length] == 0:
                        tmp1 = 1
                        tmp2 = (255, 255, 255)
                    else:
                        tmp1 = 2
                        tmp2 = (0, 0, 0)
                    self.bitmap[row][col] = tmp1
                    self.frame[row][col] = tmp2

    def image_panning(self, data, x, y):
        # TODO
        pass

    def image_rotation(self, data, z):
        # TODO
        pass

    def map_callback(self, data):
        self.image_scaling(data, self.SCALE)
