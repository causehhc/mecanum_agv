import numpy as np
import time
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import math

SCALE = 5


def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw


def pose_callback(data):
    frame = np.zeros((600, 600, 3), np.uint8)
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

    cv2.circle(frame, (int(x), int(y)), 2, (255, 255, 0))
    print(quart_to_rpy(data.pose.orientation.x,
                       data.pose.orientation.y,
                       data.pose.orientation.z,
                       data.pose.orientation.w))

    cv2.imshow('frame', frame)
    cv2.waitKey(1)


def main():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
