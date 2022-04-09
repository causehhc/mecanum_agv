import matplotlib.pyplot as plt
import numpy as np
import math


class Bezier:
    # 输入控制点，Points是一个array,num是控制点间的插补个数
    def __init__(self, Points, InterpolationNum):
        self.demension = Points.shape[1]  # 点的维度
        self.order = Points.shape[0] - 1  # 贝塞尔阶数=控制点个数-1
        self.num = InterpolationNum  # 相邻控制点的插补个数
        self.pointsNum = Points.shape[0]  # 控制点的个数
        self.Points = Points

    # 获取Bezeir所有插补点
    def getBezierPoints(self, method):
        if method == 0:
            return self.DigitalAlgo()
        if method == 1:
            return self.DeCasteljauAlgo()

    # 数值解法
    def DigitalAlgo(self):
        PB = np.zeros((self.pointsNum, self.demension))  # 求和前各项
        pis = []  # 插补点
        for u in np.arange(0, 1 + 1 / self.num, 1 / self.num):
            for i in range(0, self.pointsNum):
                PB[i] = (math.factorial(self.order) / (math.factorial(i) * math.factorial(self.order - i))) * (
                        u ** i) * (1 - u) ** (self.order - i) * self.Points[i]
            pi = sum(PB).tolist()  # 求和得到一个插补点
            pis.append(pi)
        return np.array(pis)

    # 德卡斯特里奥解法
    def DeCasteljauAlgo(self):
        pis = []  # 插补点
        for u in np.arange(0, 1 + 1 / self.num, 1 / self.num):
            Att = self.Points
            for i in np.arange(0, self.order):
                for j in np.arange(0, self.order - i):
                    Att[j] = (1.0 - u) * Att[j] + u * Att[j + 1]
            pis.append(Att[0].tolist())

        return np.array(pis)


class Line:
    def __init__(self, Points, InterpolationNum):
        self.demension = Points.shape[1]  # 点的维数
        self.segmentNum = InterpolationNum - 1  # 段数
        self.num = InterpolationNum  # 单段插补(点)数
        self.pointsNum = Points.shape[0]  # 点的个数
        self.Points = Points  # 所有点信息

    def getLinePoints(self):
        # 每一段的插补点
        pis = np.array(self.Points[0])
        # i是当前段
        for i in range(0, self.pointsNum - 1):
            sp = self.Points[i]
            ep = self.Points[i + 1]
            dp = (ep - sp) / (self.segmentNum)  # 当前段每个维度最小位移
            for i in range(1, self.num):
                pi = sp + i * dp
                pis = np.vstack((pis, pi))
        return pis


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
        matpi = bz.getBezierPoints(1)
        plt.plot(matpi[:, 0], matpi[:, 1], color='r')
        plt.show()
