#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class TrdDriver():
    def __init__(self):
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)

    def vel_callback(self,msg):
        print(msg.linear.x, msg.linear.y, msg.angular.z)

if __name__=='__main__':
    rospy.init_node('trd_driver')
    trd_driver = TrdDriver()
    rospy.spin()
