import numpy as np
import time
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

frame = np.zeros((600, 600, 3), np.uint8)
SCALE = 5


def image_scaling(data, scale):
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


def image_panning(data, x, y):
    # TODO
    pass


def image_rotation(data, z):
    # TODO
    pass


def map_callback(data):
    image_scaling(data, SCALE)  # max_resolution: 3.41333


def pose_callback(data):
    x = data.pose.pos.x
    y = data.pose.pos.y
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
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=1)
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
