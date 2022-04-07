import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np

from algorithm.plan.astar import astar


class PathInterface:
    def __init__(self):
        pass

    def abstract_map(self, maze):
        maze_new = np.zeros((600, 600, 1), np.uint8)
        for row in range(len(maze)):
            for col in range(len(maze[row])):
                if maze[row][col] == 2:
                    maze_new = 1
        return maze_new

    def find_path(self, maze, start, end):
        maze = self.abstract_map(maze)
        path = astar(maze, start, end)
        return path
