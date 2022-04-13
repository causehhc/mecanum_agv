import math

import numpy as np
import time
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

frame = np.zeros((600, 600, 3), np.uint8)
SCALE = 5

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
        self.bitmap = np.zeros((self.size[0], self.size[1], 1), np.uint8)
        self.frame = np.zeros((self.size[0], self.size[1], 3), np.uint8)

    def image_transform(self, data, scale, diff):
        """
        :param data: *
        :param scale: num
        :param diff: [v, >]
        :return:
        """
        # reshape data
        data_map = np.array(data.data, dtype=np.int16)
        map_len = data.info.height
        data_map.shape = (map_len, map_len)

        # slice data
        row_center = int((map_len / 2) + diff[0])
        col_center = int((map_len / 2) + diff[1])
        if scale < 1:
            scale = 1
        scale_len = int((map_len / scale) / 2)
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
        data_map_slc = data_map[row1:row2, col1:col2]

        # change val and type
        data_map_slc = np.where(data_map_slc != -1, data_map_slc, 166)
        data_map_slc = data_map_slc.astype(np.uint8)

        # resize
        data_map_slc = cv2.resize(data_map_slc, (self.size[0], self.size[1]), interpolation=cv2.INTER_NEAREST)

        # change val
        data_map_slc = np.where(data_map_slc != 0, data_map_slc, 255)
        data_map_slc = np.where(data_map_slc != 100, data_map_slc, 0)

        # set bitmap
        self.bitmap = data_map_slc.copy()
        self.bitmap = np.where(self.bitmap != 0, self.bitmap, 2)
        self.bitmap = np.where(self.bitmap != 255, self.bitmap, 1)
        self.bitmap = np.where(self.bitmap != 166, self.bitmap, 0)

        # expand_dims
        data_map_slc = np.expand_dims(data_map_slc, axis=2).repeat(3, axis=2)

        self.frame = data_map_slc

    def map_callback(self, data):
        t1 = time.time()
        self.image_transform(data, self.scale, self.diff)
        t2 = time.time()
        print(t2 - t1)
        cv2.imshow('frame', self.frame)
        cv2.waitKey(1)


def pose_callback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    display_center_point = 300
    origin_center_point = 1024
    pixel_distance_param = 20 * SCALE

    x *= pixel_distance_param
    y *= pixel_distance_param
    x += origin_center_point
    y += origin_center_point
    x /= (origin_center_point / display_center_point)
    y /= (origin_center_point / display_center_point)

    frame_t = frame.copy()
    # cv2.circle(frame_t, (int(x), int(y)), 2, (255, 255, 0))
    cv2.circle(frame_t, (int(y), int(x)), 2, (255, 255, 0))
    cv2.imshow('frame', frame_t)
    cv2.waitKey(1)


def main():
    tt = MapInterface("/map", (600, 600), 5, [-100, 0])
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, tt.map_callback, queue_size=1)
    # rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
