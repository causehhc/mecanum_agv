import rospy
from nav_msgs.msg import OccupancyGrid

import cv2
import numpy as np


class MapInterface:
    def __init__(self, topic_name):
        self.map = rospy.Subscriber(topic_name, OccupancyGrid, self.map_callback, queue_size=1)
        self.SCALE = 5  # max_resolution: 3.41333
        self.bitmap = np.zeros((600, 600, 1), np.uint8)

    def image_scaling(self, data, scale):
        origin_length = data.info.height
        display_length = 600
        center_point = origin_length / 2

        length_start = int(center_point - (origin_length / scale) / 2)
        length_incremental = int(origin_length / scale)

        if scale < 3.413:
            for row in range(0, length_incremental):
                for col in range(0, length_incremental):
                    if data.data[(length_start + row) * origin_length + length_start + col] == -1:
                        tmp = 0
                    elif data.data[(length_start + row) * origin_length + length_start + col] == 0:
                        tmp = 1
                    else:
                        tmp = 2
                    param = length_incremental / display_length
                    self.bitmap[int(row / param)][int(col / param)] = tmp
        else:
            for row in range(0, display_length):
                for col in range(0, display_length):
                    o_x = int(row / (display_length / length_incremental) + length_start)
                    o_y = int(col / (display_length / length_incremental) + length_start)
                    if data.data[o_x * origin_length + o_y] == -1:
                        tmp = 0
                    elif data.data[o_x * origin_length + o_y] == 0:
                        tmp = 1
                    else:
                        tmp = 2
                    self.bitmap[col][row] = tmp

    def image_panning(self, data, x, y):
        # TODO
        pass

    def image_rotation(self, data, z):
        # TODO
        pass

    def map_callback(self, data):
        self.image_scaling(data, self.SCALE)
