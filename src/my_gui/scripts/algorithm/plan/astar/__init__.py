import time

import numpy as np
import cv2
import copy
import math


def cv_show(img):
    cv2.imshow('', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def dilate_bin_image(bin_image):
    kernel = np.ones(shape=(13, 13))
    kernel_size = kernel.shape[0]
    if (kernel_size % 2 == 0) or kernel_size < 1:
        raise ValueError("kernel size must be odd and bigger than 1")
    if (bin_image.max() != 1) or (bin_image.min() != 0):
        raise ValueError("input image's pixel value must be 0 or 1")
    d_image = np.zeros(shape=bin_image.shape)
    center_move = int((kernel_size - 1) / 2)
    for i in range(center_move, bin_image.shape[0] - kernel_size + 1):
        for j in range(center_move, bin_image.shape[1] - kernel_size + 1):
            d_image[i, j] = np.min(bin_image[i - center_move:i + center_move, j - center_move:j + center_move])
    return d_image


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.pos = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.pos == other.pos


class Analyzer:
    def __init__(self, world, path_val, robot_radius, scale, r):
        self.path_list = []
        self.proc_list = []
        self.scale = scale

        self.path_val = path_val
        self.robot_radius = robot_radius
        self.costmap = copy.deepcopy(world)
        self.layer_list = []
        self._generate_costmap(world, r)

    def _generate_costmap(self, world, r):
        self.layer_list = []
        tmp = copy.deepcopy(world)
        tmp = np.array(tmp)
        tmp = tmp.astype(np.uint8)
        tmp.shape = (600, 600)

        # Binarization
        tmp = np.where(tmp == self.path_val, tmp, 0)  # tmp[row][col]!=path -> 0

        # black swell
        dilate_tmp = np.where(tmp != 1, tmp, 255)  # tmp[row][col]==1 -> 255
        param = int(r / self.scale * 15.0)
        if param < 1:
            param = 2
        dilate_tmp = cv2.erode(dilate_tmp, np.ones((param, param), dtype=np.uint8), 1)
        dilate_tmp = np.where(dilate_tmp != 255, dilate_tmp, 1)  # tmp[row][col]==255 -> 1
        dilate_map = dilate_tmp

        # Extract Contrast
        self.layer_list = np.argwhere((dilate_map - tmp) != 0)
        for item in self.layer_list:
            self.costmap[item[0]][item[1]] = 3
        return self.layer_list

    def _get_node(self, open_list):
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        return current_node, current_index

    def _is_out_range(self, node_position):
        if node_position[0] > (len(self.costmap) - self.robot_radius - 1) \
                or node_position[0] < self.robot_radius \
                or node_position[1] > (
                len(self.costmap[len(self.costmap) - self.robot_radius - 1]) - self.robot_radius - 1) \
                or node_position[1] < self.robot_radius:
            return True
        return False

    def _is_dewalkable(self, node_position):
        tmp = self.costmap[node_position[0] - self.robot_radius:node_position[0] + self.robot_radius,
              node_position[1] - self.robot_radius:node_position[1] + self.robot_radius]
        aim = ((self.robot_radius * 2) ** 2) * self.path_val
        return tmp.sum() == aim

    def _is_goal(self, current_node, end_node):
        if current_node is None:
            return False
        t_x = current_node.pos[0] - end_node.pos[0]
        t_y = current_node.pos[1] - end_node.pos[1]
        dis = math.sqrt((t_x ** 2) + (t_y ** 2))
        if dis < self.robot_radius:
            return True
        return False

    def astar(self, start, end):
        self.path_list = []
        self.proc_list = []
        # Create start and end node
        start_node = Node(None, start)
        end_node = Node(None, end)
        start_node.g = start_node.h = start_node.f = 0
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_set = set()

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:
            # Get the current node
            current_node, current_index = self._get_node(open_list)
            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_set.add(current_node.pos)

            # Found the goal
            if self._is_goal(current_node, end_node):
                cur = current_node
                while cur is not None:
                    self.path_list.append(cur.pos)
                    cur = cur.parent
                return True  # Return reversed path

            # Orientation detection & Adjacent squares
            for new_pos in [
                (0, -self.robot_radius),
                (0, self.robot_radius),
                (-self.robot_radius, 0),
                (self.robot_radius, 0),
                (-self.robot_radius, -self.robot_radius),
                (-self.robot_radius, self.robot_radius),
                (self.robot_radius, -self.robot_radius),
                (self.robot_radius, self.robot_radius)
            ]:
                node_pos = (current_node.pos[0] + new_pos[0], current_node.pos[1] + new_pos[1])
                new_node = Node(current_node, node_pos)
                if self._is_out_range(node_pos):  # Make sure within range
                    continue

                if not self._is_dewalkable(node_pos):  # Make sure walkable terrain
                    continue

                if new_node.pos in closed_set:  # Child is on the closed list
                    continue

                (x1, y1) = new_node.pos
                (x2, y2) = end_node.pos
                step_distance = math.sqrt(((new_pos[0]) ** 2) + ((new_pos[1]) ** 2))
                new_node.g = current_node.g + step_distance
                new_node.h = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
                new_node.f = new_node.g + new_node.h

                if new_node in open_list:
                    if current_node.g + step_distance < new_node.g:
                        # seems nothing do
                        pass
                    continue
                open_list.append(new_node)
                self.proc_list.append(node_pos)
        return False
