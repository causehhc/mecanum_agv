import math
import time

import numpy as np
import rospy

from algorithm.plan.astar import Analyzer
from algorithm.plan.Bezier import Bezier


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
        if self.runner is not None:
            self.runner.astar(start, end)
            return self.runner.path_list, self.runner.proc_list
        return [], []

    def find_bezier(self, pathList):
        if self.runner is not None:
            tmp = np.array(pathList)
            points = tmp.astype(np.float32)
            bz = Bezier(points, 1000)
            matpi = bz.getBezierPoints(0)
            uniques = np.unique(matpi, axis=0)
            tmp = uniques.astype(np.int32)
            return tmp
        return [], []

    def is_ok(self, pos1, pos2, param):
        if abs(pos1[0] - pos2[0]) <= param and abs(pos1[1] - pos2[1]) <= param:
            return True
        return False

    def direct_navigation(self, pose, end, remote):
        rate = rospy.Rate(50)
        while not self.is_ok((pose.x, pose.y), end, 1):
            pos = (pose.x, pose.y)
            end = end
            yaw = pose.yaw
            max_speed = 1

            # TF
            x1 = pos[0]
            y1 = pos[1]
            x2 = end[0]
            y2 = end[1]
            theta = -yaw
            x3 = ((x2 - x1) * math.cos(theta) - (y2 - y1) * math.sin(theta)) + x1
            y3 = ((x2 - x1) * math.sin(theta) + (y2 - y1) * math.cos(theta)) + y1
            max_speed_param = math.sqrt(max_speed ** 2 + max_speed ** 2)
            distance = math.sqrt(((x3 - x1) ** 2) + ((y3 - y1) ** 2))
            x = max_speed_param * ((x3 - x1) / distance)
            y = max_speed_param * ((y3 - y1) / distance)
            z = 0
            other = 0

            remote.move_cmd_send([x, y, z, other])
            print(x, y, time.time(), (pose.x, pose.y), end)
            rate.sleep()
        remote.move_cmd_send([0, 0, 0, 0])

    def find_aim(self, path_list, pose, remote):
        while len(path_list):
            aim_pos = path_list.pop()
            now_pos = (pose.x, pose.y)
            print("before", aim_pos, now_pos)
            self.direct_navigation(pose, aim_pos, remote)
            print("after", aim_pos, now_pos)
