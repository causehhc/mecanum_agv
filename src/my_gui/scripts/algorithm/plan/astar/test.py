# -*- coding: utf-8 -*-
from tkinter import *
import enum
import heapq
import time
import _thread


class PointState(enum.Enum):
    # 障碍物
    BARRIER = 'black'
    # 未使用
    UNUSED = 'white'
    # 已经加入过openlist的方格
    TRAVERSED = 'yellow'
    # 路径
    PATH = 'red'


class MiniMap:
    class Point:
        def __init__(self, x, y, f, g, father, state, rectangle):
            # x坐标
            self.x = x
            # y坐标
            self.y = y
            # f = g + h, h为预估代价，这里使用曼哈顿距离
            self.f = f
            # 从寻路起点到这个点的代价
            self.g = g
            # 父节点
            self.father = father
            # 当前点状态
            self.state = state
            # 当前点对应画布上的矩形
            self.rectangle = rectangle

        # 重写比较，方便堆排序
        def __lt__(self, other):
            if self.f < other.f:
                return True
            else:
                return False

    def __init__(self, *args):
        # 行数
        self.row = args[0]
        # 列数
        self.col = args[1]
        # 方格尺寸
        self.size = args[2]
        # 起点
        self.start = args[3]
        # 终点
        self.end = args[4]
        # 每次绘制的延迟时间
        self.delay = args[5]

        self.root = Tk()
        self.root.title("navigation")
        self.canva = Canvas(self.root, width=self.col * self.size + 3, height=self.row * self.size + 3)
        # 生成方格集合
        self.points = self.generatePoints()
        # 生成网格
        self.generateMesh()

        self.canva.bind("<Button-1>", self.createBarrier)
        self.canva.bind("<Button-2>", self.cleanMap)
        self.canva.bind("<Button-3>", self.navigation)

        self.canva.pack(side=TOP, expand=YES, fill=BOTH)
        self.root.resizable(0, 0)
        self.root.mainloop()

    def generatePoints(self):
        """
        初始化绘制用的方格集合
        设置每个方格的状态和对应的矩形
        """
        points = [[self.Point(x, y, 0, 0, None, PointState.UNUSED.value, \
                              self.canva.create_rectangle((x * self.size + 3, y * self.size + 3),
                                                          ((x + 1) * self.size + 3, (y + 1) * self.size + 3),
                                                          fill=PointState.UNUSED.value)) \
                   for y in range(self.row)] for x in range(self.col)]
        return points

    def generateMesh(self):
        """
        绘制网格
        """
        for i in range(self.row + 1):
            self.canva.create_line((3, i * self.size + 3), (self.col * self.size + 3, i * self.size + 3))
        for i in range(self.col + 1):
            self.canva.create_line((i * self.size + 3, 3), (i * self.size + 3, self.row * self.size + 3))

    def createBarrier(self, event):
        """
        设置障碍/移除障碍
        通过鼠标点击位置更改对应方格的状态
        """
        x = int((event.x + 3) / self.size)
        y = int((event.y + 3) / self.size)
        if x < self.col and y < self.row:
            if (self.points[x][y].state == PointState.BARRIER.value):
                self.points[x][y].state = PointState.UNUSED.value
                self.canva.itemconfig(self.points[x][y].rectangle, fill=self.points[x][y].state)
            else:
                self.points[x][y].state = PointState.BARRIER.value
                self.canva.itemconfig(self.points[x][y].rectangle, fill=self.points[x][y].state)

    def cleanMap(self, event):
        """
        清空画布
        """
        for i in range(self.col):
            for j in range(self.row):
                # 不清空障碍物（因为太难点了），不需要此功能则去掉判断
                if (self.points[i][j].state != PointState.BARRIER.value):
                    self.points[i][j].state = PointState.UNUSED.value
                    self.canva.itemconfig(self.points[i][j].rectangle, fill=self.points[i][j].state)

    def navigation(self, event):
        """
        新开一个线程调用寻路函数
        """
        _thread.start_new_thread(self.generatePath, (self.start, self.end))

    def generatePath(self, start, end):
        """
        开始寻路
        """
        x1 = start[0]
        y1 = start[1]
        x2 = end[0]
        y2 = end[1]

        # 用最小堆存点，使每次取出的点都预估代价最小
        openlist = []
        # 两个set用于查找，存储坐标的二元组
        closeset = set()
        openset = set()
        # 将起点加入openlist,每格距离设置为10，是为了使斜着走时距离设置为14，方便计算
        heapq.heappush(openlist, self.points[x1][y1])
        openset.add((x1, y1))
        # 寻路循环
        temp = None
        while 1:
            # 从openlist中取出代价最小点
            pMin = heapq.heappop(openlist)
            openset.remove((pMin.x, pMin.y))
            # 将这个点放入closelist中
            closeset.add((pMin.x, pMin.y))
            # 遍历八个方向
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if i == 0 and j == 0: continue
                    # 当前要判断的点的坐标
                    xNew = pMin.x + i
                    yNew = pMin.y + j
                    # 如果这个点越界则跳过
                    if xNew >= self.col or xNew < 0 or yNew >= self.row or yNew < 0:
                        continue
                    pNew = self.points[xNew][yNew]
                    # 这个点是否在pMin的非正方向（即四个角落）
                    isCorner = i != 0 and j != 0
                    if isCorner:
                        # 如果将要判断的斜角方向被阻挡则跳过（如将要判断东南方向，若东方或南方被阻挡，则跳过）
                        if self.points[xNew][pMin.y].state == PointState.BARRIER.value or self.points[pMin.x][yNew].state == PointState.BARRIER.value:
                            continue
                    # 如果在closeset中出现或该点为障碍物则跳过判断
                    if (xNew, yNew) not in closeset and self.points[xNew][yNew].state != PointState.BARRIER.value:
                        # 如果在openset中
                        if (xNew, yNew) in openset:
                            # 如果通过起点到pMin再到pNew的代价比起点到pNew的代价小，则更新pNew的代价，将pMin设置为pNew的父节点
                            if ((14 if isCorner else 10) + pMin.g) < pNew.g:
                                # 如果在角落，则pMin到pNew的代价为14，否则为10
                                pNew.g = pMin.g + (14 if isCorner else 10)
                                pNew.f = pNew.g + 10 * (abs(x2 - xNew) + abs(y2 - yNew))
                                pNew.father = pMin
                        # 如果不在openset中，则代表这个点第一次被访问，直接将pMin设置为pNew的父节点
                        else:
                            # 如果在角落，则pMin到pNew的代价为14，否则为10
                            pNew.g = pMin.g + (14 if isCorner else 10)
                            pNew.f = pNew.g + 10 * (abs(x2 - xNew) + abs(y2 - yNew))
                            pNew.father = pMin
                            pNew.state = PointState.TRAVERSED.value
                            self.canva.itemconfig(pNew.rectangle, fill="blue")
                            if temp is not None:
                                self.canva.itemconfig(temp, fill="yellow")
                            temp = pNew.rectangle
                            # 将这个点加入openlist
                            heapq.heappush(openlist, pNew)
                            openset.add((xNew, yNew))
            # 检测是否寻路完成
            if (x2, y2) in openset:
                print('sb')
                pNext = self.points[x2][y2]
                pNext.state = PointState.PATH.value
                self.canva.itemconfig(pNext.rectangle, fill=PointState.PATH.value)
                while pNext.father:
                    pNext = pNext.father
                    self.points[pNext.x][pNext.y].state = PointState.PATH.value
                    self.canva.itemconfig(self.points[pNext.x][pNext.y].rectangle, fill=PointState.PATH.value)
                break
            # 如果寻路不完成但openlist长度为0，则没有可达路径
            if (len(openlist) == 0):
                print('Unreachable!')
                break
            # 等待绘制
            time.sleep(self.delay)


if __name__ == '__main__':
    # 参数为行数，列数，方格尺寸,起点坐标，终点坐标，延迟时间
    demo = MiniMap(25, 30, 20, (0, 0), (29, 24), 0.02)
