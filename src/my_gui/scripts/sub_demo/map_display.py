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
            data_map = np.array(data.data, dtype=np.int16)
            map_len = int(math.sqrt(len(data_map)))
            data_map.shape = (map_len, map_len)
            x1 = int(map_len/2-600/2)
            x2 = int(map_len/2+600/2)

            # data_map_scl = data_map[x1:x2][x1:x2]
            data_map_scl = np.zeros((600, 600), dtype=np.int16)
            t1 = time.time()
            for row in range(0, 600):
                for col in range(0, 600):
                    data_map_scl[row][col] = data_map[x1+row][x1+col]
            t2 = time.time()
            print(t2 - t1)

            self.frame = np.expand_dims(data_map_scl, axis=2).repeat(3, axis=2)
            self.frame = np.where(self.frame != [-1, -1, -1], self.frame, [166, 166, 166])
            self.frame = np.where(self.frame != [0, 0, 0], self.frame, [255, 255, 255])
            self.frame = np.where(self.frame != [100, 100, 100], self.frame, [0, 0, 0])
            self.frame = self.frame.astype(np.uint8)

            # out_put = np.where(data_map_scl != -1, data_map_scl, (166, 166, 166))  # out_put[row][col]==-1 -> (166, 166, 166)
            # out_put = np.where(data_map_scl != 0, data_map_scl,(255, 255, 255))  # out_put[row][col]==-1 -> (166, 166, 166)
            # out_put = np.where(data_map_scl ==-1 and data_map_scl ==0, data_map_scl,(0, 0, 0))  # out_put[row][col]==-1 -> (166, 166, 166)
            # # map_len = cv2.resize(data_map, (600, 600), interpolation=cv2.INTER_CUBIC)
            # self.frame = out_put

            # for row in range(0, output_len):
            #     for col in range(0, output_len):
            #         o_x = int(row / (output_len / len_incre) + len_start)
            #         o_y = int(col / (output_len / len_incre) + len_start)
            #         data_index = o_x+o_y*input_len
            #         if data.data[data_index] == -1:
            #             tmp1 = 0
            #             tmp2 = (166, 166, 166)
            #             self.bitmap[row][col] = tmp1
            #             self.frame[row][col] = tmp2
            #         elif data.data[data_index] == 0:
            #             tmp1 = 1
            #             tmp2 = (255, 255, 255)
            #             self.bitmap[row][col] = tmp1
            #             self.frame[row][col] = tmp2
            #         else:
            #             tmp1 = 2
            #             tmp2 = (0, 0, 0)
            #             self.bitmap[row][col] = tmp1
            #             self.frame[row][col] = tmp2

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
        # print(t2-t1)
        cv2.imshow('frame', self.frame)
        cv2.waitKey(1)



def pose_callback(data):
    x = data.pose.position.x
    y = data.pose.position.y
    display_center_point = 300
    origin_center_point = 1024
    pixel_distance_param = 20*SCALE

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
    tt = MapInterface("/map", (600, 600), 5)
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, tt.map_callback, queue_size=1)
    # rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
