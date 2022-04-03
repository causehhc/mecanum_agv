import numpy as np
import time
import rospy
import cv2
from sensor_msgs.msg import CompressedImage


def callback(data):
    frame = np.fromstring(data.data, np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    cv2.imshow('frame', frame)
    cv2.waitKey(1)


def main():
    rospy.init_node('cam_listener', anonymous=True)
    rospy.Subscriber("/sim/smallCar/camera/image_raw/compressed", CompressedImage, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
