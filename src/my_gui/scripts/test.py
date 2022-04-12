import _thread
import time
from multiprocessing import Process

import cv2
import numpy as np

from algorithm.plan.Bezier import Bezier


def main():
    a = [[0, 1, 2], [1, 2, 0], [2, 0, 1]]
    c = [[[0, 0, 0], [1, 1, 1], [2, 2, 2]], [[1, 1, 1], [2, 2, 2], [0, 0, 0]], [[2, 2, 2], [0, 0, 0], [1, 1, 1]]]
    a = np.array(a)
    c = np.array(c)

    b = np.expand_dims(a, axis=2).repeat(3, axis=2)  # a->c
    print(b)
    b = np.where(b != [1, 1, 1], b, [166, 166, 166])  # out_put[row][col]==-1 -> (166, 166, 166)
    print(b)

    sb = np.zeros((300, 300, 3), dtype=np.uint8)
    sb = np.where(sb != [0, 0, 0], sb, [255, 255, 255])
    sb = sb.astype(np.uint8)
    while True:
        cv2.imshow('frame', sb)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()
