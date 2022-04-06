import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np

from algorithm.plan.astar import astar


class PathInterface:
    def __init__(self):
        self.frame = np.zeros((600, 600, 3), np.uint8)
        self.path = []

    def abstract_map(self, frame):
        maze = 0
        return maze

    def find_path(self, maze, start, end):
        maze = self.abstract_map(maze)
        self.path = astar(maze, start, end)

    def clear_path(self):
        self.path = []

    def creat_path_img(self):
        for item in self.path:
            self.frame[item[0]][item[1]] = (250, 0, 255)

    def get_frame_qt(self):
        return cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

    def move_to_aim(self):
        pass


def main():
    rospy.init_node('path_listener')
    path = PathInterface()
    while True:
        frame = path.frame
        if frame is not None:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
