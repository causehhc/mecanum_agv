import time

import rospy
from nav_msgs.msg import OccupancyGrid

import cv2
import numpy as np


class MapInterface:
    def __init__(self, topic_name, size, scale):
        self.size = size
        self.map = rospy.Subscriber(topic_name, OccupancyGrid, self.map_callback, queue_size=1)
        self.SCALE = scale  # max_resolution: 3.41333
        self.bitmap = np.zeros((self.size[0], self.size[1], 1), np.uint8)
        self.frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)

    def image_scaling(self, data, scale):
        input_len = data.info.height
        output_len = self.size[0]
        center_point = input_len / 2

        len_start = int(center_point - (input_len / scale) / 2)
        len_incre = int(input_len / scale)

        if scale < 3.413:
            # TODO
            for row in range(0, len_incre):
                for col in range(0, len_incre):
                    data_index = (len_start + row)+(len_start + col) * input_len
                    res_row = int(row / (len_incre / output_len))
                    res_col = int(col / (len_incre / output_len))
                    if data.data[data_index] == -1:
                        tmp1 = 0  # UNKNOWN
                        tmp2 = (166, 166, 166)
                        self.bitmap[res_row][res_col] = tmp1
                        self.frame[res_row][res_col] = tmp2
                    elif data.data[data_index] == 0:
                        tmp1 = 1  # PATH
                        tmp2 = (255, 255, 255)
                        self.bitmap[res_row][res_col] = tmp1
                        self.frame[res_row][res_col] = tmp2
                    else:
                        tmp1 = 2  # WALL
                        tmp2 = (0, 0, 0)
                        self.bitmap[res_row][res_col] = tmp1
                        self.frame[res_row][res_col] = tmp2
        else:
            # TODO
            for row in range(0, output_len):
                for col in range(0, output_len):
                    o_x = int(row / (output_len / len_incre) + len_start)
                    o_y = int(col / (output_len / len_incre) + len_start)
                    data_index = o_x+o_y*input_len
                    if data.data[data_index] == -1:
                        tmp1 = 0
                        tmp2 = (166, 166, 166)
                        self.bitmap[row][col] = tmp1
                        self.frame[row][col] = tmp2
                    elif data.data[data_index] == 0:
                        tmp1 = 1
                        tmp2 = (255, 255, 255)
                        self.bitmap[row][col] = tmp1
                        self.frame[row][col] = tmp2
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
        t1 = time.time()
        self.image_scaling(data, self.SCALE)
        t2 = time.time()
        print(t2-t1)
