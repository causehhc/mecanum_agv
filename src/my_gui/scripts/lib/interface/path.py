import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np

from algorithm.plan.astar import astar


class PathInterface:
    def __init__(self):
        pass

    def find_path(self, maze, start, end):
        path = []
        print(start, end, maze[start[0]][start[1]], maze[end[0]][end[1]])
        path = astar(maze, start, end)
        return path
