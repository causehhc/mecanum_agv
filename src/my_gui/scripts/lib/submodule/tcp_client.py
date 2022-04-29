from socket import *
import os


class Client:
    def __init__(self):
        self.BUFFSIZE = 1024
        self.tcp_client_socket = socket(AF_INET, SOCK_STREAM)

    def connect(self, ipaddr, port):
        HOST = ipaddr
        PORT = port
        ADDRESS = (HOST, PORT)
        self.tcp_client_socket.connect(ADDRESS)

    def send_msg(self, msg):
        data = msg
        self.tcp_client_socket.send(data.encode('utf-8'))

    def close(self):
        self.tcp_client_socket.close()

    def recv_msg(self):
        data = self.tcp_client_socket.recv(self.BUFFSIZE)
        data = data.decode("utf-8")
        return data


def main():
    client = Client()
    # client.connect('127.0.0.1', 15057)
    client.connect('192.168.2.184', 15057)
    while True:
        data = input('>')
        client.send_msg(data)
        if data == 'exit':
            client.close()
            break
        print(client.recv_msg())


if __name__ == '__main__':
    main()
