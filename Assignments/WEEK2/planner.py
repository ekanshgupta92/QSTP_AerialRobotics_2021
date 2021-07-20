import numpy as np
from matplotlib import colors 
import matplotlib.pyplot as plt

class Node:
    def __init__(self,x , y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

        self.g = 0
        self.h = 0
        self.f = 0

def path_plan(current_node, maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append([current.x, current.y])
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    print(path)
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1
    return result

def search(start, goal, maze):
    cost = 1 # cost for travelling basically the g value :p

    start_node = Node(start[0], start[1], None)
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(goal[0], goal[1], None)
    goal_node.g = goal_node.h = goal_node.f = 0

    yet_to_visit_list = []  
    visited_list = [] 
    
    yet_to_visit_list.append(start_node)

    move  =  [[-1, 0 ], # up
              [ 0, -1], # left
              [ 1, 0 ], # down
              [ 0, 1 ]] # right

    no_rows, no_columns = np.shape(maze)
    # Loop until you find the end

    while len(yet_to_visit_list) > 0:  
        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0

        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            return path_plan(current_node,maze)

        children = []
        for adjacent_pos in move: 
            node_position = [current_node.x + adjacent_pos[0], current_node.y + adjacent_pos[1]]

            if (node_position[0] > (no_rows - 1) or node_position[0] < 0 or 
                node_position[1] > (no_columns -1) or node_position[1] < 0):
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(node_position[0], node_position[1], current_node)
            children.append(new_node)

        for child in children:
            # already visited node so skipping that 
            if child in visited_list:  
                continue

            child.g = current_node.g + cost
            child.h = (((child.x - goal_node.x) ** 2) + ((child.y - goal_node.y) ** 2)) 
            child.f = child.g + child.h

            # if child is already in yet_to_visit_list child cost is more 
            # ie the current path to that node is longer and skip that
            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue

            yet_to_visit_list.append(child)


if __name__ == '__main__':
    """ 
        1 means obstacle and 0 means clear path that bot can take.
        In the plot, the red is the calculated path, black the obastacels and blue the available nodes
    """

    # maze = [[0, 1, 0, 0, 0, 0],
    #         [0, 1, 0, 0, 0, 0],
    #         [0, 1, 0, 1, 0, 0],
    #         [0, 1, 0, 0, 1, 0],
    #         [0, 0, 0, 0, 1, 0]]
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    
    start = [0, 0] 
    end = [7,6] 

    no_rows, no_columns = np.shape(maze)
    path = search(start, end, maze)
    print(path)

    for i in range(no_rows):
        for j in range(no_columns):
            if path[i][j] != -1 :
                maze[i][j] = -1

cmap = colors.ListedColormap(['red','blue', 'black'])
plt.figure(figsize=(6,6))
plt.pcolor(maze[::-1],cmap=cmap,edgecolors='k', linewidths=3)
plt.show()