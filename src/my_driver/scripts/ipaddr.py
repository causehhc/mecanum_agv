#!/usr/bin/env python
import rospy
import serial
import socket

def get_host_ip():
    """
    查询本机ip地址
    :return: ip
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
        return ip


if __name__=='__main__':
    rospy.init_node('ipaddr')
    rate = rospy.Rate(1)
    ser = serial.Serial('/dev/ttyUSB_stm32', 115200)
    while not rospy.is_shutdown():
        ip = get_host_ip()
        cmd_str = "{}{}{}".format('"', ip, '"')
        cmd = list(cmd_str)
        print(cmd)
        ser.write(cmd)
        rate.sleep()
