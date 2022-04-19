import numpy as np
import time
import rospy
import cv2
from sensor_msgs.msg import CompressedImage


def callback(data):
    frame = np.fromstring(data.data, np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    origin_size = frame.shape
    aim_size = 600
    param = aim_size/origin_size[1]
    now_size = (int(origin_size[1]*param), int(origin_size[0]*param))
    frame1 = cv2.resize(frame, now_size)
    cv2.imshow('frame', frame1)
    cv2.waitKey(1)


def main():
    rospy.init_node('cam_listener', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
