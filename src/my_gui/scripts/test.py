import _thread
import time
from multiprocessing import Process


class Test:
    def __init__(self):
        self.wtf = [[0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]]

    def test1(self, haha=1):
        while True:
            # print(1)
            self.wtf = [[1, 1, 1],
                        [1, 1, 1],
                        [1, 1, 1]]
            time.sleep(1)

    def test2(self, haha=1):
        while True:
            # print(2)
            self.wtf = [[1, 0, 1],
                        [1, 0, 1],
                        [1, 0, 1]]
            time.sleep(1)

    def test3(self, haha=1):
        while True:
            # print(3)
            self.wtf = [[1, 1, 1],
                        [0, 0, 0],
                        [1, 1, 1]]
            time.sleep(1)


def main():
    a = Test()
    _thread.start_new_thread(a.test1, (1,))
    _thread.start_new_thread(a.test2, (1,))
    _thread.start_new_thread(a.test3, (1,))
    while True:
        print(a.wtf)
        time.sleep(1)


if __name__ == '__main__':
    main()
