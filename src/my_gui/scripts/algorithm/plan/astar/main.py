import copy
import numpy as np
from tkinter import *
import _thread
import time

from matplotlib import pyplot as plt

from algorithm.plan.astar import Analyzer

robot_radius = 5


class Visual:
    def __init__(self, maze, start, end, PATH):
        self.maze = maze
        self.start = start
        self.end = end
        self.PATH = PATH

        t1 = time.time()
        self.runner = Analyzer(maze, PATH, robot_radius, 5)
        t2 = time.time()
        print(t2 - t1)

        t1 = time.time()
        self.runner.astar(self.start, self.end)
        t2 = time.time()
        print(t2 - t1)
        self.path_list = self.runner.path_list
        self.proc_list = self.runner.proc_list

        # plt.imshow(self.runner.costmap)
        # plt.show()

        self.root = Tk()
        self.root.title("navigation")
        self.canva = Canvas(self.root, width=len(self.runner.costmap), height=len(self.runner.costmap[0]))
        self.draw_background(1)
        self.canva.bind("<Button-1>", self.startFunc)
        self.canva.pack(side=TOP, expand=YES, fill=BOTH)
        self.root.mainloop()

    def startFunc(self, event):
        # _thread.start_new_thread(self.runner.astar, (self.start, self.end))
        # _thread.start_new_thread(self.runner.astar, (self.end, self.start))
        _thread.start_new_thread(self.draw_process, (1,))

    def draw_background(self, haha=1):
        for row in range(len(self.runner.costmap)):
            for col in range(len(self.runner.costmap[0])):
                ret = self.runner.costmap[int(row)][int(col)]
                x = col
                y = row
                if ret == self.PATH:
                    self.canva.create_rectangle(x, y, x, y, outline="white")
                elif ret == 3:
                    self.canva.create_rectangle(x, y, x, y, outline="blue")
                elif ret == 0:
                    self.canva.create_rectangle(x, y, x, y, outline="grey")
                elif ret == 2:
                    self.canva.create_rectangle(x, y, x, y, outline="black")
        self.canva.create_rectangle(
            self.start[1] - robot_radius / 2,
            self.start[0] - robot_radius / 2,
            self.start[1] + robot_radius / 2,
            self.start[0] + robot_radius / 2,
            outline="black",
            fill="red"
        )
        self.canva.create_rectangle(
            self.end[1] - robot_radius / 2,
            self.end[0] - robot_radius / 2,
            self.end[1] + robot_radius / 2,
            self.end[0] + robot_radius / 2,
            outline="black",
            fill="red"
        )

    def draw_process(self, haha=1):
        tmp_x = 0
        tmp_y = 0
        proc_list = copy.deepcopy(self.proc_list)
        while len(proc_list):
            # time.sleep(0.02)
            node = proc_list.pop(0)
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

        path_list = copy.deepcopy(self.path_list)
        while len(path_list):
            node = path_list.pop(0)
            self.canva.create_rectangle(
                node[1] - robot_radius / 2,
                node[0] - robot_radius / 2,
                node[1] + robot_radius / 2,
                node[0] + robot_radius / 2,
                outline="black",
                fill="green"
            )



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
    # maze = np.fromfile("./maze.bin", dtype=np.int8)
    maze.shape = 600, 600, 1

    start = (400, 110)
    end = (300, 120)
    # haoshi
    start = (400, 110)
    end = (300, 120)

    return maze, start, end, 1


def main():
    maze, start, end, path = demo2()

    Visual(maze, start, end, path)

    # tt = Analyzer(maze, path, robot_radius)
    # tt.astar(start, end)
    # print(tt.path_list)
    # print(tt.proc_list)


if __name__ == '__main__':
    main()
