#!/usr/bin/env python
import serial
import socket
import time

def get_host_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
        return ip


if __name__=='__main__':
    ser = serial.Serial('/dev/ttyUSB_stm32', 115200)
    while True:
        ip = get_host_ip()
        cmd_str = "{}{}{}".format('"', ip, '"')
        cmd = bytes(cmd_str, encoding = "utf8")
        ser.write(cmd)
        time.sleep(10)