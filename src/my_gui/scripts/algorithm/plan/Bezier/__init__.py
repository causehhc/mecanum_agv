import time

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
