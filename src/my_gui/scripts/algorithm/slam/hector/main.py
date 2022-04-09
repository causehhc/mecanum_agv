from algorithm.slam.hector.HectorMappingRos import HectorMappingRos
import rospy
from sensor_msgs.msg import LaserScan


def main():
    sm = HectorMappingRos()
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("/sim/smallCar/laser/scan", LaserScan, sm.scanCallback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
