import time


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
    def __init__(self):
        self.path_list = []
        self.proc_list = []

    def get_node(self, open_list):
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        return current_node, current_index

    def is_out_range(self, maze, node_position):
        if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
            return True
        return False

    def is_dewalkable(self, maze, node_position, PATH):
        if maze[node_position[0]][node_position[1]] != PATH:
            return True
        return False

    def astar(self, maze, start, end, PATH):
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
        tmp = None
        while len(open_list) > 0:
            print(len(open_list))

            # Get the current node
            current_node, current_index = self.get_node(open_list)
            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                current = current_node
                while current is not None:
                    self.path_list.append(current.pos)
                    current = current.parent
                return True  # Return reversed path

            # Orientation detection
            for new_pos in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares
                # Get node position
                node_pos = (current_node.pos[0] + new_pos[0], current_node.pos[1] + new_pos[1])
                # Create new node
                new_node = Node(current_node, node_pos)

                # Make sure within range
                if self.is_out_range(maze, node_pos):
                    continue

                #  Make sure walkable terrain
                if self.is_dewalkable(maze, node_pos, PATH):
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
