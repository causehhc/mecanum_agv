import time

import rospy
from nav_msgs.msg import OccupancyGrid

import cv2
import numpy as np


class MapInterface:
    def __init__(self, topic_name, size, scale, diff):
        self.size = size
        self.scale = scale
        self.diff = diff
        self.map = rospy.Subscriber(topic_name, OccupancyGrid, self.map_callback, queue_size=1)
        self.data = None
        self.bitmap = np.zeros((self.size[0], self.size[1], 1), np.uint8)
        self.frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)

    def _get_two_from_oneDim(self, arr1, arr1_rowLen, row1, col1, row2, col2):
        arr2 = arr1[row1*arr1_rowLen:(row2)*arr1_rowLen]
        arr2 = np.array(arr2, dtype=np.int16)
        arr2.shape = (row2-row1, arr1_rowLen)
        arr2 = arr2[:, col1:col2]
        return arr2

    def image_transform(self):
        """
        :param data: *
        :return:
        """
        if self.data is None:
            return False

        data_tmp = self.data.data
        map_len = self.data.info.height

        # slice data
        row_center = int((map_len / 2) + self.diff[0])
        col_center = int((map_len / 2) + self.diff[1])
        if self.scale < 1:
            self.scale = 1
        scale_len = int((map_len / self.scale) / 2)
        if row_center - scale_len < 0:
            row_center = scale_len
        elif row_center + scale_len >= map_len:
            row_center = map_len - scale_len
        if col_center - scale_len < 0:
            col_center = scale_len
        elif col_center + scale_len >= map_len:
            col_center = map_len - scale_len
        row1 = row_center - scale_len
        col1 = col_center - scale_len
        row2 = row_center + scale_len
        col2 = col_center + scale_len
        data_map_slc = self._get_two_from_oneDim(data_tmp, map_len, row1, col1, row2, col2)

        # change val and type
        data_map_slc = np.where(data_map_slc != -1, data_map_slc, 166)
        data_map_slc = data_map_slc.astype(np.uint8)

        # resize
        data_map_slc = cv2.resize(data_map_slc, (self.size[0], self.size[1]), interpolation=cv2.INTER_NEAREST)

        # change val
        data_map_slc = np.where(data_map_slc != 0, data_map_slc, 255)
        data_map_slc = np.where(data_map_slc != 100, data_map_slc, 0)

        # T
        data_map_slc = data_map_slc.T

        # set bitmap
        bitmap_tmp = data_map_slc.copy()
        bitmap_tmp = np.where(bitmap_tmp != 0, bitmap_tmp, 2)
        bitmap_tmp = np.where(bitmap_tmp != 255, bitmap_tmp, 1)
        bitmap_tmp = np.where(bitmap_tmp != 166, bitmap_tmp, 0)

        # expand_dims
        data_map_slc = np.expand_dims(data_map_slc, axis=2).repeat(3, axis=2)

        # res
        self.frame = data_map_slc
        self.bitmap = bitmap_tmp


    def change_param(self, scale, diff):
        """
        :param scale: num
        :param diff: [v, >]
        :return:
        """
        self.scale += scale
        self.diff[0] += diff[0]
        self.diff[1] += diff[1]

    def map_callback(self, data):
        self.data = data
        self.image_transform()

