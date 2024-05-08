import cv2
import numpy as np
import math
import random
import matplotlib.pyplot as plt

###############################################################
'''
This implementation of the RRT* algorithm is based upon code written by Aakash Yadav. 
Link: https://github.com/nimRobotics/RRT/tree/master

The code has been heavily modified to work with this project.
Code written by Markus Rekkedal
'''
##############################################################


GREEN = (0, 255, 0)     # RGB values for green color
BLACK = (0, 0, 0)       # RGB values for black color
BLUE = (0, 0, 255)      # RGB values for blue color
RED = (255, 0, 0)       # RGB values for red color


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)    # Distance between two nodes

def get_random_point(end, bias, height, width): 
    if random.random() > bias: 
        return Node(random.uniform(0, width), random.uniform(0, height)) # Chooses a random point in space if random value is larger than bias value both being 0-1
    else:
        return end                                                       # Chooses the endpoint if random value is smaller than bias value
    
def nearest_node(nodes, point):
    return min(nodes, key=lambda n: distance(n, point))                  # Finds the closest neighbouring node to the chosen node, n = node in list "nodes"

def new_node(q_near, q_rand, delta):
    dist = distance(q_near, q_rand)                                      # q_near: nearest node to q_rand
    if dist < delta:                                                     # checks if new node is within delta-reach 
        return q_rand
    else:
        theta = math.atan2(q_rand.y - q_near.y, q_rand.x - q_near.x) 
        return Node(q_near.x + delta * math.cos(theta), q_near.y + delta * math.sin(theta)) # If new node is further away than delta it is shortened to delta distance away from original node

def get_points_on_line(x1, y1, x2, y2):                                  # Bresenham's Line Algorithm implementation to get points along a line
    points = []                                                          # Takes in two points, finds all points on the line between them
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    sx = -1 if x1 > x2 else 1
    sy = -1 if y1 > y2 else 1

    if dx > dy:
        err = dx / 2
        while x != x2:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != y2:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x, y))
    return points

def collision_check(q_near, q_new, obstacle_image, height, width):
    x_near, y_near = int(q_near.x), int(q_near.y)                        # Retrieves x and y values for q_near and q_new
    x_new, y_new = int(q_new.x), int(q_new.y)

    points_on_line = get_points_on_line(x_near, y_near, x_new, y_new)    # Use Bresenham's Line Algorithm to get all points along the line

    for x, y in points_on_line:                                          # Check for collision along each point on the line
        if not (0 <= x < width and 0 <= y < height):                     # If the point is outside the image boundaries, it's a collision          
            return False     
        pixel_color = obstacle_image[y, x]                               # If the pixel is black, it's a collision
        if np.all(pixel_color == [0, 0, 0]):
            return False       
    return True                                                          # If no collision detected along the line, return True

def extract_path(path, target_x, target_y, start, end, obstacle_image):                # Find the node in the path list that has the same coordinates as end
    for node in path:
        if node.x == target_x and node.y == target_y:
            if node:
                #print('costa de noda: ', node.cost)
                path = retrace_path(node, start, end, obstacle_image)
                return(path, node.cost)
            else:
                print("No path found, try again")
            return node
    return None    

def retrace_path(end_node, start, end, obstacle_image): # Extract the coordinates from the optimal path
    # Trace back the optimal path from the end node
        best_path = []
        current_node = end_node
        iteration = 0 # Initialize iteration count
        while current_node:
            best_path.insert(0, current_node)
            current_node = current_node.parent
            #print('backstep iteration: ', iteration)
            iteration += 1  # Increment iteration count
            if (current_node == start) or iteration > 50:  # Exit the loop when the start node is reached or program iterates to infinity
                break
        path = []
        current = end_node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        # Visualize the optimal path    
        #image_with_nodes = draw_nodes(best_path, start, end, iteration, obstacle_image )
        #cv2.waitKey(1000)
        #cv2.destroyAllWindows()   
        return path[::-1]                                                 

def rrt_star_alg(start, end, max_iterations, delta, radius, bias, obstacle_image, height, width):
    nodes = [start]                                                      # Initiates at start node
    reached_end = False                                                  # Flag to track if the end node has been reached
    for iteration in range(max_iterations):                              # Runs the specified amount of iterations
        if reached_end:
            break  # Exit the loop if the end node has been reached
        
        q_rand = get_random_point(end, bias, height, width)              # chooses a random point in space if random value is larger than bias value, both being 0-1, if not it chooses end
        q_near = nearest_node(nodes, q_rand)                             # Finds the closest neighbouring node to q_rand
        q_new = new_node(q_near, q_rand, delta)                          # new node is shortened to be within delta distance of q_rand

        near_nodes = [node for node in nodes if distance(node, q_new) < radius]
        sorted_nodes = sorted(near_nodes, key=lambda node: distance(node, q_new))

        for candidate_node in sorted_nodes:
            if collision_check(q_near, q_new, obstacle_image, height, width):         # Checks if there is a collision on the line between q_near and q_new
                near_nodes = [node for node in nodes if distance(node, q_new) < radius] # Checks which neighbouring nodes are closest to q_new within radius
                q_min = q_near                                                          # Updates q_min to the closest neighbouring node
                c_min = q_near.cost + distance(q_near, q_new)                           # lowest cost path found from q_near to q_min

                for near_node in near_nodes:                                                # iterates the loop for each "near_node"
                    if collision_check(near_node, q_new, obstacle_image, height, width):    # Checks if collision, if no collision occurs proceeds
                        c_near = near_node.cost + distance(near_node, q_new)                # Calculates the cost of a path from q_near to q_min
                        if c_near < c_min:                                                  # If c_near is less than c_min, c_min is updated to c_near since it is a shorter path 
                            q_min = near_node
                            c_min = c_near

                if q_min != q_new:
                    if collision_check(q_min, q_new, obstacle_image, height, width):                                               
                        q_new.parent = q_min                                          # Gives the new node the parentnode with the shortest path
                        q_new.cost = c_min                                            # Updates the cost of the new node to the min_cost found associated with the parentnode
                        nodes.append(q_new)                                           # Add q_new to list of nodes expanding the RRT* tree

                        if distance(q_new, end) < delta:                              # Check if the new node is the end node
                            reached_end = True
                            end.cost = q_new.cost + distance(q_new, end)              # Update the cost of the end node
                            end.parent = q_new                                        # Update the parent node of the end node
                            nodes.append(end)                                         # Add the end node to the list of nodes  

                else:
                    closest_node = min(nodes, key=lambda node: node.cost + distance(node, q_new))
                    if closest_node != q_new:                                                       # Check if the closest node is not the same as q_new
                        if collision_check(closest_node, q_new, obstacle_image, height, width): 
                            q_new.parent = closest_node
                            q_new.cost = closest_node.cost + distance(closest_node, q_new)
                            nodes.append(q_new)

                            if distance(q_new, end) < delta:
                                reached_end = True
                                end.cost = q_new.cost + distance(q_new, end)                         # Update the cost of the end node
                                end.parent = q_new                                                   # Update the parent node of the end node
                                nodes.append(end)                                                    # Add the end node to the list of nodes
            break                                                                                    # Exit the loop after finding the first node that passes the collision check

        # Visualization
        #draw_nodes(nodes, start, end, iteration, obstacle_image)
    #cv2.destroyAllWindows()
      
    return nodes

def draw_nodes(nodes, start, end, iteration, obstacle_image):
    image = np.copy(obstacle_image)                                               # Makes a copy of the original picture
    for node in nodes:    
        image = cv2.circle(image, (int(node.x), int(node.y)), 3, BLUE, -1)       # Places the generated nodes in space and colors them blue
        if node.parent:
            image = cv2.line(image, (int(node.x), int(node.y)), (int(node.parent.x), int(node.parent.y)), BLACK, 1) # If a node has a parent node, a black line is made to connect them
    image = cv2.circle(image, (int(start.x), int(start.y)), 5, GREEN, -1)         # Places the start location as a circle in space and colors it green 
    image = cv2.circle(image, (int(end.x), int(end.y)), 5, RED, -1)            # Places the end location as a circle in space and colors it red
    cv2.putText(image, f"Iterations: {iteration}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2) # Shows which iteration the algorithm is at
    cv2.imshow("RRT* Path Planning", cv2.resize(image, (720, 720)))                                       # Displays the image
    cv2.waitKey(1)
    return image                                                                  # Returrrtstarn the modified image

#Main function    
def rrtstar(occupancy_grid, start_coords, end_coords):
 
    start = Node(start_coords[0],start_coords[1])
    end = Node(end_coords[0],end_coords[1])        # Adjust the coordinates of the end point as needed
    max_iterations = 1500                          # Number of iterations
    num_run = 5                                    # Number of paths to generate, chooses the one with lowest costs, runs the algorithm this many times
    bias = 0.05                                    # Bias towards end
    delta = 90                                     # Stepsize
    radius = 300                                   # Radius looking for closer parent node
    

    inverted_grid = 1 - occupancy_grid # inverting 1's and 0's land, making sure land will be black and sea will be white
    cv2.imwrite('data/RRTSTAR_obstcl.png', np.uint8(255 * inverted_grid)) # Converts occupancy grid to image format

    obstacle_image = cv2.imread("data/RRTSTAR_obstcl.png") # Loads the obstacle image

    height, width, _ = obstacle_image.shape   # Retrieves height and shape from image


    # Extract the optimal path
    min_cost = float('inf')  # Initialize minimum cost

    for i in range(num_run):
        path = rrt_star_alg(start, end, max_iterations, delta, radius, bias, obstacle_image, height, width) # Runs the RRTSTAR algorithm with all parameters
        print('RRT* has  done its job', (i+1), '/', num_run,' times')  
        RRT_pot_path, path_cost = extract_path(path, end.x, end.y, start, end, obstacle_image)   # Retraces path from endnode back to start, returns waypoints and cost of path
        print('Path cost:', path_cost)
        
        if path_cost < min_cost:                                                                            # Updates min_cost if a path with lower cost is generated
                min_cost = path_cost
                print('New lowest cost', min_cost)
                RRT_PATH = RRT_pot_path                                                                     # Updates RRT_PATH if a shorter path is found


    print('Optimal path coordinates: ', RRT_PATH)
    return RRT_PATH
