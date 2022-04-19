#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import serial


class TrdDriver():
    def __init__(self):
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        self.ser = serial.Serial('/dev/ttyUSB_stm32', 115200)

    def vel_callback(self,msg):
        print(msg.linear.x, msg.linear.y, msg.angular.z)
        cmd = [0x5b, 0x64, 0x64, 0x64, 0x5d]
        tx = 100+msg.linear.x*10
        ty = 100+msg.linear.y*10
        tz = 100+msg.angular.z*20
        cmd[1] = chr(int(tx))
        cmd[2] = chr(int(ty))
        cmd[3] = chr(int(tz))
        self.ser.write(cmd)


if __name__=='__main__':
    rospy.init_node('trd_driver')
    trd_driver = TrdDriver()
    rospy.spin()
