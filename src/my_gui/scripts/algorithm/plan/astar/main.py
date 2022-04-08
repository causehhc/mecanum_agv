import numpy as np
from tkinter import *
import _thread
import time

from matplotlib import pyplot as plt

from algorithm.plan.astar import Analyzer


class Visual:
    def __init__(self, maze, start, end, PATH):
        self.runner = Analyzer()

        self.maze = maze
        self.start = start
        self.end = end
        self.PATH = PATH

        self.path_list = self.runner.path_list
        self.proc_list = self.runner.proc_list

        self.root = Tk()
        self.root.title("navigation")
        self.row = len(maze)
        self.col = len(maze[0])
        self.canva = Canvas(self.root, width=self.col, height=self.row)
        for row in range(self.row):
            for col in range(self.col):
                ret = maze[int(row)][int(col)]
                x = col
                y = row
                if ret != PATH:
                    self.canva.create_rectangle(x, y, x, y, outline="black")
        self.canva.create_rectangle(start[1], start[0], start[1], start[0], outline="red")
        self.canva.create_rectangle(end[1], end[0], end[1], end[0], outline="red")

        self.canva.bind("<Button-1>", self.startFunc)

        self.canva.pack(side=TOP, expand=YES, fill=BOTH)
        self.root.mainloop()

    def draw(self, hahah):
        tmp_x = 0
        tmp_y = 0
        while True:
            if len(self.path_list):
                node = self.path_list.pop(0)
                self.canva.create_rectangle(
                    node[1],
                    node[0],
                    node[1],
                    node[0],
                    outline="green"
                )
            if len(self.proc_list):
                node = self.proc_list.pop(0)
                self.canva.create_rectangle(
                    node[1],
                    node[0],
                    node[1],
                    node[0],
                    outline="blue"
                )
                self.canva.create_rectangle(
                    tmp_y,
                    tmp_x,
                    tmp_y,
                    tmp_x,
                    outline="yellow"
                )
                tmp_x = node[0]
                tmp_y = node[1]
            # time.sleep(0.02)

    def startFunc(self, event):
        _thread.start_new_thread(self.runner.astar, (self.maze, self.start, self.end, self.PATH))
        # _thread.start_new_thread(self.runner.astar, (self.maze, self.end, self.start, self.PATH))
        # _thread.start_new_thread(self.draw, (1,))


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
    return maze, start, end, 0


def demo2():
    maze = np.fromfile("/home/hhc/Desktop/ros/bishe_ws/src/my_gui/scripts/algorithm/plan/astar/maze.bin", dtype=np.int8)
    maze.shape = 600, 600, 1

    start = (310, 220)
    end = (370, 220)

    return maze, start, end, 1


def main():
    maze, start, end, path = demo2()
    # plt.imshow(maze)
    # plt.show()

    # Visual(maze, start, end, path)

    tt = Analyzer()
    tt.astar(maze, start, end, path)
    print(tt.path_list)


if __name__ == '__main__':
    main()
