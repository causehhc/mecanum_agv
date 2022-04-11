import math
import time

import rospy

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

    def is_ok(self, pos1, pos2, param):
        if abs(pos1[0] - pos2[0]) <= param and abs(pos1[1] - pos2[1]) <= param:
            return True
        return False

    def direct_navigation(self, pose, end, remote):
        rate = rospy.Rate(50)
        while not self.is_ok((pose.x, pose.y), end, 1):
            x1 = pose.x
            y1 = pose.y

            x2 = end[0]
            y2 = end[1]

            max_speed = 1
            param = math.sqrt(max_speed**2+max_speed**2)
            distance = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
            x = param * ((x2 - x1) / distance)
            y = param * ((y2 - y1) / distance)

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
