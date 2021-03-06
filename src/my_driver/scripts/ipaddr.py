#!/usr/bin/env python
import serial
from socket import *
import time

def get_host_ip():
    s = socket(AF_INET, SOCK_DGRAM)
    s.connect(('8.8.8.8', 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


if __name__=='__main__':
    ser = serial.Serial('/dev/ttyUSB_stm32', 115200)
    cnt = 0
    while True:
        ip = get_host_ip()
        cmd_str = "{}{}:{}{}".format('"',cnt, ip, '"')
        cmd = bytes(cmd_str, encoding = "utf8")
        ser.write(cmd)
        print(cmd)
        cnt += 1
        if cnt==10:
          cnt=0
        time.sleep(1)