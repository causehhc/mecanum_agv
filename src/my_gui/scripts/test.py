import _thread
import time
from multiprocessing import Process

import cv2
import numpy as np

from algorithm.plan.Bezier import Bezier


def test1():
    a = [[0, 1, 2], [1, 2, 0], [2, 0, 1]]
    c = [[[0, 0, 0], [1, 1, 1], [2, 2, 2]], [[1, 1, 1], [2, 2, 2], [0, 0, 0]], [[2, 2, 2], [0, 0, 0], [1, 1, 1]]]
    a = np.array(a)
    c = np.array(c)
    print(a[0] == c[0][0])

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

def test2():
    a = [[0, 1, -1, 0, 1, -1],
         [1, -1, 0, 0, 1, -1],
         [-1, 0, 1, 0, 1, -1],
         [-1, 0, 1, 0, 1, -1],
         [-1, 0, 1, 0, 1, -1],
         [-1, 0, 1, 0, 1, -1]]
    a = np.array(a)
    a = np.where(a != -1, a, 166)
    a = a.astype(np.uint8)
    # print(a)
    b = cv2.resize(a, (3, 3), interpolation=cv2.INTER_NEAREST)
    b = np.expand_dims(b, axis=2).repeat(3, axis=2)
    print(b)
    print('sb')
    print(b.T)

def test3(pixel_distance_param, row_center, col_center, display_center_point):
    x = 300
    y = 300
    x *= pixel_distance_param
    y *= pixel_distance_param
    x += row_center
    y += col_center
    x /= (row_center / display_center_point)
    y /= (col_center / display_center_point)
    return int(x), int(y)


def main():
    print(test3(20, 10, 10, 300))
    print(test3(20, 11, 10, 300))


if __name__ == '__main__':
    main()
