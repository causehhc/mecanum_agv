import matplotlib.pyplot as plt
import numpy as np

from algorithm.plan.Bezier import *

if __name__ == '__main__':
    # points = np.array([
    #     [1, 3, 0],
    #     [1.5, 1, 0],
    #     [4, 2, 0],
    #     [4, 3, 4],
    #     [2, 3, 11],
    #     [5, 5, 9]
    # ])

    bin = [(410, 220), (410, 225), (410, 230), (410, 235), (410, 240), (410, 245), (410, 250), (410, 255), (410, 260),
           (405, 265), (400, 265), (395, 265), (390, 265), (385, 265), (380, 270), (375, 275), (370, 280), (365, 285),
           (360, 290), (355, 290), (350, 290), (345, 290), (340, 290), (335, 290), (330, 285), (330, 280), (330, 275),
           (325, 270), (320, 270), (315, 270), (310, 270), (305, 270), (300, 270), (295, 270), (290, 265), (290, 260),
           (290, 255), (290, 250), (290, 245), (295, 240), (300, 235), (305, 230), (310, 225), (310, 220)]

    temp = []
    for item in bin:
        temp.append([float(item[1]), float(item[0])])
    points = np.array(temp)
    print(points.shape[1])

    # points = np.array([
    #     [0.0, 0.0],
    #     [1.0, 0.0],
    #     [2.0, 0.0],
    #     [3.0, 1.0],
    # ])
    # print(points.shape[1])

    if points.shape[1] == 3:

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # 标记控制点
        for i in range(0, points.shape[0]):
            ax.scatter(points[i][0], points[i][1], points[i][2], marker='o', color='r')
            ax.text(points[i][0], points[i][1], points[i][2], i, size=12)

        # 直线连接控制点
        l = Line(points, 1000)
        pl = l.getLinePoints()
        ax.plot3D(pl[:, 0], pl[:, 1], pl[:, 2], color='k')

        # 贝塞尔曲线连接控制点
        bz = Bezier(points, 1000)
        matpi = bz.getBezierPoints(0)
        ax.plot3D(matpi[:, 0], matpi[:, 1], matpi[:, 2], color='r')
        plt.show()
    if points.shape[1] == 2:

        # 标记控制点
        for i in range(0, points.shape[0]):
            plt.scatter(points[i][0], points[i][1], marker='o', color='r')
            plt.text(points[i][0], points[i][1], i, size=12)

        # 直线连接控制点
        l = Line(points, 1000)
        pl = l.getLinePoints()
        plt.plot(pl[:, 0], pl[:, 1], color='k')

        # 贝塞尔曲线连接控制点
        bz = Bezier(points, 1000)
        matpi = bz.getBezierPoints(1)  # too slow
        plt.plot(matpi[:, 0], matpi[:, 1], color='r')
        plt.show()
