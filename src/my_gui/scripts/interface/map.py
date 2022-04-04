import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np


def image_scaling(data, scale, frame):
    origin_length = data.info.height
    display_length = 600
    center_point = origin_length / 2

    length_start = int(center_point - (origin_length / scale) / 2)
    length_incremental = int(origin_length / scale)

    if scale < 3.413:
        for row in range(0, length_incremental):
            for col in range(0, length_incremental):
                if data.data[(length_start + row) * origin_length + length_start + col] == -1:
                    wtf = (166, 166, 166)
                elif data.data[(length_start + row) * origin_length + length_start + col] == 0:
                    wtf = (250, 255, 255)
                else:
                    wtf = (0, 0, 0)
                param = length_incremental / display_length
                # frame[int(row / param)][int(col / param)] = wtf
                frame[int(col / param)][int(row / param)] = wtf
    else:
        for row in range(0, display_length):
            for col in range(0, display_length):
                o_x = int(row / (display_length / length_incremental) + length_start)
                o_y = int(col / (display_length / length_incremental) + length_start)
                if data.data[o_x * origin_length + o_y] == -1:
                    wtf = (166, 166, 166)
                elif data.data[o_x * origin_length + o_y] == 0:
                    wtf = (250, 255, 255)
                else:
                    wtf = (0, 0, 0)
                # frame[row][col] = wtf
                frame[col][row] = wtf
    return frame


def image_panning(data, x, y, frame):
    # TODO
    return frame


def image_rotation(data, z, frame):
    # TODO
    return frame


class MapInterface:
    def __init__(self, topic_name1, topic_name2):
        self.map = rospy.Subscriber(topic_name1, OccupancyGrid, self.map_callback, queue_size=1)
        self.pose = rospy.Subscriber(topic_name2, PoseStamped, self.pose_callback, queue_size=1)
        self.frame = np.zeros((600, 600, 3), np.uint8)
        self.frame_t = self.frame.copy()
        self.SCALE = 5

    def map_callback(self, data):
        self.frame = image_scaling(data, self.SCALE, self.frame)  # max_resolution: 3.41333

    def pose_callback(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        display_center_point = 300
        origin_center_point = 1024
        pixel_distance_param = 20 * self.SCALE

        x *= pixel_distance_param
        y *= pixel_distance_param
        x += origin_center_point
        y += origin_center_point
        x /= (origin_center_point / display_center_point)
        y /= (origin_center_point / display_center_point)

        self.frame_t = self.frame.copy()
        # cv2.circle(frame_t, (int(x), int(y)), 2, (255, 255, 0))
        cv2.circle(self.frame_t, (int(y), int(x)), 2, (255, 255, 0))

    def get_frame_qt(self):
        return cv2.cvtColor(self.frame_t, cv2.COLOR_BGR2RGB)


def main():
    rospy.init_node('map_listener')
    map = MapInterface("/map", "/slam_out_pose")
    while True:
        frame = map.frame_t
        if frame is not None:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
