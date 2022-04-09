import numpy as np
from tkinter import *
import _thread
import time

from matplotlib import pyplot as plt

from algorithm.plan.astar import Analyzer

from algorithm.plan.astar.BezierPath import Bezier

robot_radius = 5


class Visual:
    def __init__(self, maze, start, end, PATH):
        self.runner = Analyzer(maze, PATH, robot_radius)

        self.maze = maze
        self.start = start
        self.end = end
        self.PATH = PATH

        self.path_list = self.runner.path_list
        self.proc_list = self.runner.proc_list

        # plt.imshow(self.runner.costmap)
        # plt.show()

        self.root = Tk()
        self.root.title("navigation")
        self.row = len(self.runner.costmap)
        self.col = len(self.runner.costmap[0])
        self.canva = Canvas(self.root, width=self.col, height=self.row)
        for row in range(self.row):
            for col in range(self.col):
                ret = self.runner.costmap[int(row)][int(col)]
                x = col
                y = row
                if ret == PATH:
                    self.canva.create_rectangle(x, y, x, y, outline="white")
                elif ret == 3:
                    self.canva.create_rectangle(x, y, x, y, outline="blue")
                elif ret == 0:
                    self.canva.create_rectangle(x, y, x, y, outline="grey")
                elif ret == 2:
                    self.canva.create_rectangle(x, y, x, y, outline="black")

        self.canva.create_rectangle(
            start[1] - robot_radius / 2,
            start[0] - robot_radius / 2,
            start[1] + robot_radius / 2,
            start[0] + robot_radius / 2,
            outline="black",
            fill="red"
        )
        self.canva.create_rectangle(
            end[1] - robot_radius / 2,
            end[0] - robot_radius / 2,
            end[1] + robot_radius / 2,
            end[0] + robot_radius / 2,
            outline="black",
            fill="red"
        )

        self.canva.bind("<Button-1>", self.startFunc)

        self.canva.pack(side=TOP, expand=YES, fill=BOTH)
        self.root.mainloop()

    def draw(self, hahah):
        tmp_x = 0
        tmp_y = 0
        points = []

        while len(self.proc_list):
            node = self.proc_list.pop(0)
            self.canva.create_rectangle(
                node[1] - robot_radius / 2,
                node[0] - robot_radius / 2,
                node[1] + robot_radius / 2,
                node[0] + robot_radius / 2,
                outline="black",
                fill="blue"
            )
            self.canva.create_rectangle(
                tmp_y - robot_radius / 2,
                tmp_x - robot_radius / 2,
                tmp_y + robot_radius / 2,
                tmp_x + robot_radius / 2,
                outline="black",
                fill="yellow"
            )
            tmp_x = node[0]
            tmp_y = node[1]

        while len(self.path_list):
            node = self.path_list.pop(0)
            points.append([float(node[1]), float(node[0])])
            self.canva.create_rectangle(
                node[1] - robot_radius / 2,
                node[0] - robot_radius / 2,
                node[1] + robot_radius / 2,
                node[0] + robot_radius / 2,
                outline="black",
                fill="green"
            )

        if len(points) != 0:
            points = np.array(points)
            bz = Bezier(points, 1000)
            matpi = bz.getBezierPoints(1)
            nb = list(matpi)
            while len(nb):
                node = nb.pop(0)
                self.canva.create_rectangle(
                    int(node[0]),
                    int(node[1]),
                    int(node[0]),
                    int(node[1]),
                    outline="red"
                )

        time.sleep(0.02)

    def startFunc(self, event):
        _thread.start_new_thread(self.runner.astar, (self.start, self.end))
        # _thread.start_new_thread(self.runner.astar, (self.maze, self.end, self.start, self.PATH))
        _thread.start_new_thread(self.draw, (1,))


def demo1():
    maze = [[0, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (2, 7)
    maze = np.array(maze)
    maze.shape = 10, 10, 1
    return maze, start, end, 0


def demo2():
    maze = np.fromfile("/home/hhc/Desktop/ros/bishe_ws/src/my_gui/scripts/algorithm/plan/astar/maze.bin", dtype=np.int8)
    maze.shape = 600, 600, 1

    start = (310, 220)
    end = (310, 300)

    return maze, start, end, 1


def main():
    maze, start, end, path = demo2()
    # plt.imshow(maze)
    # plt.show()

    sb = Visual(maze, start, end, path)

    # tt = Analyzer()
    # tt.astar(maze, start, end, path)
    # print(tt.path_list)


if __name__ == '__main__':
    main()
