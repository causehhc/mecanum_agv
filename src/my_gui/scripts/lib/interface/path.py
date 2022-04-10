import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np

from algorithm.plan.astar import Analyzer


class PathInterface:
    def __init__(self, maze, robot_radius):
        self.maze = maze
        self.PATH = 1
        self.robot_radius = robot_radius
        self.runner = None

    def find_map(self):
        self.runner = Analyzer(self.maze, self.PATH, self.robot_radius)
        return self.runner.layer_list

    def find_path(self, start, end):
        self.runner.astar(start, end)
        return self.runner.path_list, self.runner.proc_list
