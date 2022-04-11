import numpy as np
import cv2


def cv_show(img):
    cv2.imshow('', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test1():
    tmp = cv2.imread('./test.jpg')
    (B, G, R) = cv2.split(tmp)  # 提取R、G、B分量
    tmp = B
    for row in range(len(tmp)):
        for col in range(len(tmp[0])):
            tmp[row][col] = 0
    cv_show(tmp)
    print(tmp.max(), tmp.min())
    print(type(tmp), tmp.shape)
    dilate_map = cv2.dilate(tmp, kernel=np.ones((13, 13), dtype=np.uint8))
    cv_show(dilate_map)


def test2():
    tmp = [[0, 1, 2],
           [1, 2, 0],
           [2, 3, 1]]

    tmp = np.array(tmp)
    tmp.sum()
    tmp = tmp[1:2+1, 0:1+1]
    print(tmp)
    print(3 in tmp)
    # tmp = np.array(tmp)
    # dilate_tmp = np.where(tmp != 1, tmp, 255)  # 1->255
    # print(dilate_tmp)


def main():
    test2()


if __name__ == '__main__':
    main()
