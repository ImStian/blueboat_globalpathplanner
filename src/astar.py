import matplotlib.pyplot as plt
import math as m
from shapely.geometry import LineString
import numpy as np
import heapq
from warnings import warn

###############################################################
'''
This implementation of the A* algorithm is based upon
pseudocode and documentation from an article on Medium.com written by
Nicholas Swift. Link: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2

Modified and implemented into this project by Stian Bernhardsen
'''
##############################################################

def visualize_search(grid, closedlist,size):
    ''' Visualizing progress of pathfinding. Displays the contents of the closedlist.'''
    grid_with_path = np.rot90(np.flip(np.array(grid),0),3) # Flipping & Rotating the map 270* to make it match the  coordinates system
    x = []
    y = []
    for node in closedlist:
        x.append(node.position[0])
        y.append(node.position[1])
    plt.scatter(x,y, s=0.5)
    plt.xlim(0,size[0])
    plt.ylim(0,size[1])
    plt.imshow(grid_with_path, cmap='Greys')
    plt.savefig("./data/%s.png"%'pciture', dpi=300, bbox_inches='tight', pad_inches=0)
    plt.close()


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.child = None

        # Cost values (default = 0)
        self.g = 0
        self.h = 0
        self.f = 0
        
        # Decimation Flag
        self.include_in_path = 1

    # Functions for cost-comparison:
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __gt__(self, other):
        return self.f > other.f
    
    # Debugging (Not necessairy for functionality):
    def __repr__(self):
        return f"{self.position} - g: {self.g}, h: {self.h}, f: {self.f}"


def backtrack_path(current_node, check_inclusion=False):
    path = []
    previous_node = None
    while current_node.parent is not None:
        current_node.child = previous_node # Storing child data within node

        if check_inclusion and (current_node.include_in_path == 1):
            path.append(current_node.position)
        elif not check_inclusion:
            path.append(current_node)

        previous_node = current_node # Updating child
        current_node = current_node.parent # Moving to the parent node
    return path

def get_all_children(node):
    children_list = []
    current_node = node
    while node.child is not None:
        children_list.append(current_node.child) # Adding the child node
        current_node = node.child # Moving to the child node
    return children_list



def decimation_filter(path, epsillon=2.0):
    path_as_linestring = LineString(path)
    return path_as_linestring.simplify(epsillon, preserve_topology=True).coords

                    
            


def generate_children(parent, grid):
    '''Returns a list of child nodes generated from a parent, taking into account the occupancy grid.'''
    children = []
    current_position = parent.position


    for displacement in [(0, 1), (0, -1), (1, 0), (-1, 0)]: # Only 
        new_position = (current_position[0] + displacement[0],  current_position[1] + displacement[1])

        # Out out bounds?
        if new_position[0] > (len(grid) - 1) or new_position[0] < 0 or new_position[1] > (len(grid[len(grid)-1]) -1) or new_position[1] < 0:
            continue # Next child
 
        # In water?
        if grid[new_position[0]][new_position[1]] != 0:
            continue # Next child

        # Making child node
        child = Node(parent, new_position)
        children.append(child)

    return children


def euclidean_distance_sqrd(start, end):
    ''' Returns the euclidean distance squared between two points. Squared for performance reasons. '''
    return ((start[0]- end[0])**2 + (start[1] - end[1])**(2))


def astar(grid, start_position, end_position,size):
    counter = 0
    # Defining start- and end nodes:
    start_node = Node(None, start_position)
    end_node   = Node(None, end_position)
    
    # Initializing the open and closed list
    open_list   = []
    closed_list = []

    heapq.heapify(open_list)             # Making openlist a priority queue
    heapq.heappush(open_list,start_node) # Adding start to open list

    # Running the A* algorithm:
    while  len(open_list) > 0:
        current_node = heapq.heappop(open_list)      # Get node with lowest 'f' value from open list
        closed_list.append(current_node)             # Add it to the closed list


        # Checking if at goal
        if current_node == end_node:
            decimated_path = decimation_filter(backtrack_path(current_node, check_inclusion=True))
            return decimated_path[::-1]  # Returning path


        # Generating children at adjacent nodes
        children = generate_children(current_node, grid)

        # Looping through children
        for child in children:

            # Skip if child in closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Creating f, g and h values:
            child.g = current_node.g + 1
            child.h = euclidean_distance_sqrd(child.position, end_node.position)
            child.f = child.g + child.h

            # Skip if g-higher than nodes with the same position
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Adding child to the open list
            heapq.heappush(open_list,child)

            if counter % 500 == 0:
                visualize_search(grid,closed_list, size)
            counter += 1