#!/usr/bin/env python
from socket import *
from time import ctime
import os
import subprocess
import roslaunch


class Driver:
    def __init__(self):
        self.launch_file = "/home/pi/ros/mecanum_agv/src/my_driver/launch/start.launch"
        self.tracking_launch = None
        self.status = False
    def config(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.launch_file])
    def start(self):
        self.tracking_launch.start()
        self.status = True
    def stop(self):
        self.tracking_launch.shutdown()
        self.status = False



def get_host_ip():
    s = socket(AF_INET, SOCK_DGRAM)
    s.connect(('8.8.8.8', 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


class Server:
    def __init__(self):
        self.BUFFSIZE = 1024
        self.tcp_server_socket = socket(AF_INET, SOCK_STREAM)

    def create_server(self, ipaddr, port):
        HOST = ipaddr
        PORT = port

        ADDR = (HOST, PORT)
        self.tcp_server_socket.bind(ADDR)
        self.tcp_server_socket.listen(5)

    def start(self):
        driver = Driver()
        while True:
            print('waiting for connecting...')
            tcp_client_socket, addr = self.tcp_server_socket.accept()
            print('connected from:', addr)
            while True:
                try:
                    data = tcp_client_socket.recv(self.BUFFSIZE)
                    data = data.decode("utf-8")
                    data_list = data.split(' ')
                    if data == 'exit':
                        break
                    elif data == 'start':
                        if driver.status == False:
                            driver.config()
                            driver.start()
                        else:
                            print("Has Start")
                    elif data == 'status':
                        print(driver.status)
                    elif data == 'stop':
                        if driver.status == True:
                            driver.stop()
                        else:
                            print("Has No Start")
                    elif data_list[0] == "IP":
                        os.environ["ROS_IP"] = data_list[1]
                    elif data_list[0] == "MASTER":
                        os.environ["ROS_MASTER_URI"] = data_list[1]
                    else:
                        os.system(data)
                    print('[{}][{}]: {}'.format(ctime(), tcp_client_socket.getpeername(), data))
                    tcp_client_socket.send(bytes('ok', encoding="utf-8"))
                except:
                    tcp_client_socket.send(bytes('no', encoding="utf-8"))
                


def main():
    server = Server()
    # server.create_server('127.0.0.1', 15057)
    server.create_server(get_host_ip(), 15057)
    server.start()
        


if __name__ == '__main__':
    main()