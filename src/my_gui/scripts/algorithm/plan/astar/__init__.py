import time
import numpy as np
import copy
import math


def dilate_bin_image(bin_image):
    kernel = np.ones(shape=(13, 13))
    kernel_size = kernel.shape[0]
    bin_image = np.array(bin_image)
    if (kernel_size%2 == 0) or kernel_size<1:
        raise ValueError("kernel size must be odd and bigger than 1")
    if (bin_image.max() != 1) or (bin_image.min() != 0):
        raise ValueError("input image's pixel value must be 0 or 1")
    d_image = np.zeros(shape=bin_image.shape)
    center_move = int((kernel_size-1)/2)
    for i in range(center_move, bin_image.shape[0]-kernel_size+1):
        for j in range(center_move, bin_image.shape[1]-kernel_size+1):
            d_image[i, j] = np.min(bin_image[i-center_move:i+center_move,j-center_move:j+center_move])
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
    def __init__(self, world, path_val, robot_radius):
        self.path_list = []
        self.proc_list = []

        self.path_val = path_val
        self.robot_radius= robot_radius
        self.costmap = copy.deepcopy(world)

        self.generate_costmap(world)


    def generate_costmap(self, world):
        tmp = copy.deepcopy(world)
        for row in range(len(tmp)):
            for col in range(len(tmp[0])):
                if tmp[row][col] != self.path_val:
                    tmp[row][col] = 0
        dilate_map = dilate_bin_image(tmp)

        for row in range(len(dilate_map)):
            for col in range(len(dilate_map[0])):
                if dilate_map[row][col] != tmp[row][col]:
                    self.costmap[row][col] = 3


    def _get_node(self, open_list):
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        return current_node, current_index

    def _is_out_range(self, node_position):
        if node_position[0] > (len(self.costmap) - self.robot_radius-1) \
                or node_position[0] < self.robot_radius \
                or node_position[1] > (len(self.costmap[len(self.costmap) -self.robot_radius- 1])-self.robot_radius - 1)\
                or node_position[1] < self.robot_radius:
            return True
        return False

    def _is_dewalkable(self, node_position):
        for row in range(node_position[0]-self.robot_radius, node_position[0]+self.robot_radius):
            for col in range(node_position[1] - self.robot_radius, node_position[1] + self.robot_radius):
                if self.costmap[row][col] != self.path_val:
                    return False
        return True
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
        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:
            # print(len(open_list))
            # print(len(closed_list))

            # Get the current node
            current_node, current_index = self._get_node(open_list)
            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if self._is_goal(current_node, end_node):
                current = current_node
                while current is not None:
                    self.path_list.append(current.pos)
                    current = current.parent
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
                # Get node position
                node_pos = (current_node.pos[0] + new_pos[0], current_node.pos[1] + new_pos[1])
                # Create new node
                new_node = Node(current_node, node_pos)

                # Make sure within range
                if self._is_out_range(node_pos):
                    continue

                #  Make sure walkable terrain
                if not self._is_dewalkable(node_pos):
                    continue

                # Child is on the closed list
                if new_node in closed_list:
                    continue

                # Create the f, g, and h values
                new_node.g = current_node.g + 1
                new_node.h = ((new_node.pos[0] - end_node.pos[0]) ** 2) + ((new_node.pos[1] - end_node.pos[1]) ** 2)
                new_node.f = new_node.g + new_node.h

                # Child is already in the open list
                if new_node in open_list:
                    idx = open_list.index(new_node)
                    if new_node.g > open_list[idx].g:
                        continue

                # Add the child to the open list
                open_list.append(new_node)
                self.proc_list.append(node_pos)
        print('Unreachable!')
        return False