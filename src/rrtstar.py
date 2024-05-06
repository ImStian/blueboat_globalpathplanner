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
PURPLE = (128, 0, 128)  # RGB values for purple color

# Define Node class
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

# Define RRT* algorithm
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

def is_collision_free(q_near, q_new, obstacle_image, height, width):
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

def find_path_from_end_node(path, target_x, target_y, start, end, obstacle_image):                # Find the node in the path list that has the same coordinates as end
    for node in path:
        if node.x == target_x and node.y == target_y:
            if node:
                #print('costa de noda: ', node.cost)
                path = optimal_path(node, start, end, obstacle_image)
                return(path, node.cost)
            else:
                print("No path found, try again scrub")
            return node
    return None    

def optimal_path(end_node, start, end, obstacle_image): # Extract the coordinates from the optimal path
    # Trace back the optimal path from the end node
        optimal_path = []
        current_node = end_node
        iteration = 0# Initialize iteration count
        while current_node:
            optimal_path.insert(0, current_node)
            current_node = current_node.parent
            #print('backstep iteration: ', iteration)
            iteration += 1  # Increment iteration count
            if (current_node == start) or iteration > 50:  # Exit the loop when the start node is reached or program iterates to infinity
                print('we got all the way home')
                break

        # Print the x and y coordinates of each node in the optimal path
        #print("Optimal Path:")
        path = []
        current = end_node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        # Visualize the optimal path    
        image_with_nodes = draw_nodes(optimal_path, start, end, iteration, obstacle_image )
        cv2.waitKey(1000)
        cv2.destroyAllWindows()   
        return path[::-1] 
        # Visualize the optimal path                                                   

def rrt_star_alg(start, end, max_iterations, delta, radius, bias, obstacle_image, height, width):
    nodes = [start]                                                      # Initiates at start node
    for iteration in range(max_iterations):                              # Runs the specified amount of iterations
        q_rand = get_random_point(end, bias, height, width)              # chooses a random point in space if random value is larger than bias value, both being 0-1, if not it chooses end
        q_near = nearest_node(nodes, q_rand)                             # Finds the closest neighbouring node to q_rand
        q_new = new_node(q_near, q_rand, delta)                          # new node is shortened to be within delta distance of q_rand
        #print('iteration: ', iteration)

        if is_collision_free(q_near, q_new, obstacle_image, height, width):         # Checks if there is a collision on the line between q_near and q_new
            near_nodes = [node for node in nodes if distance(node, q_new) < radius] # Checks which neighbouring nodes are closest to q_new within radius
            q_min = q_near                                                          # Updates q_min to the closest neighbouring node
            c_min = q_near.cost + distance(q_near, q_new)                           # lowest cost path found from q_near to q_min
            #print(c_min)

            for near_node in near_nodes:                                                # iterates the loop for each "near_node"
                if is_collision_free(near_node, q_new, obstacle_image, height, width):  # Checks if collision, if no collision occurs proceeds
                    c_near = near_node.cost + distance(near_node, q_new)                # Calculates the cost of a path from q_near to q_min
                    if c_near < c_min:                                                  # If c_near is less than c_min, c_min is updated to c_near since it is a shorter path 
                        q_min = near_node
                        c_min = c_near

            if q_min != q_new:
                if is_collision_free(q_min, q_new, obstacle_image, height, width):                                               
                    q_new.parent = q_min                                          # Gives the new node the parentnode with the shortest path
                    q_new.cost = c_min                                            # Updates the cost of the new node to the min_cost found associated with the parentnode
                    nodes.append(q_new)                                           # Add q_new to list of nodes expanding the RRT* tree

            else:
                closest_node = min(nodes, key=lambda node: node.cost + distance(node, q_new))
                if closest_node != q_new:  # Check if the closest node is not the same as q_new
                    if is_collision_free(closest_node, q_new, obstacle_image, height, width): 
                        q_new.parent = closest_node
                        q_new.cost = closest_node.cost + distance(closest_node, q_new)

            nodes.append(q_new)  

        # Visualization
        draw_nodes(nodes, start, end, iteration, obstacle_image)

    cv2.destroyAllWindows()
      
    return nodes

def draw_nodes(nodes, start, end, iteration, obstacle_image):
    image = np.copy(obstacle_image)                                               # Makes a copy of the original picture
    for node in nodes:    
        image = cv2.circle(image, (int(node.x), int(node.y)), 3, GREEN, -1)       # Places the generated nodes in space and colors them green
        if node.parent:
            image = cv2.line(image, (int(node.x), int(node.y)), (int(node.parent.x), int(node.parent.y)), BLACK, 1) # If a node has a parent node, a black line is made to connect them
    image = cv2.circle(image, (int(start.x), int(start.y)), 5, GREEN, -1)         # Places the start loaction as a cirle in space and colors it green 
    image = cv2.circle(image, (int(end.x), int(end.y)), 5, PURPLE, -1)            # Places the end loaction as a cirle in space and colors it purple
    cv2.putText(image, f"Iterations: {iteration}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2) # Shows wich iteration the algorhitm is at
    
    cv2.imshow("RRT* Path Planning", cv2.resize(image, (481, 481)))                                       # Displays the image
    cv2.waitKey(1)
    return image                                                                  # Return the modified image

#Main function
def rrtstar(occupancy_grid, start_coords, end_coords):
 
    start = Node(start_coords[0],start_coords[1])
    end = Node(end_coords[0],end_coords[1])    # Adjust the coordinates of the end point as needed
    max_iterations = 850   # Number of iterations
    num_run = 1             # Number of paths to generate, chooses the one with lowest costs, runs the algorhitm this many times
    bias = 0.1              # Bias towards end
    delta = 30               # Stepsize
    radius = 150             # Radius looking for closer parent node
    

    inverted_grid = 1 - occupancy_grid # inverting 1's and 0's land, making sure land will be black and sea will be white
    cv2.imwrite('data/RRTSTAR_obstcl.png', np.uint8(255 * inverted_grid)) # Converst occupancy grid to image format

    obstacle_image = cv2.imread("data/RRTSTAR_obstcl.png") # Loads the obstacle image
    #print(obstacle_image)
    height, width, _ = obstacle_image.shape   # Retrieves height and shape from image
    #print('heigh: ', height,'width: ', width)

    # Extract the optimal path
    min_cost = float('inf')  # Initialize minimum cost

    for i in range(num_run):
        path = rrt_star_alg(start, end, max_iterations, delta, radius, bias, obstacle_image, height, width) # Runs the RRTSTAR algorhitm with all parameters
        print('RRT* has  done its job', (i+1), '/', num_run,' times')  
        RRT_pot_path, path_cost = find_path_from_end_node(path, end.x, end.y, start, end, obstacle_image)   # Retraces path from endnode back to start, returns waypoints and cost of path
        print('Path cost:', path_cost)
        
        if path_cost < min_cost:                                                                            # Updates min_cost if a path with lower cost is generated
                min_cost = path_cost
                print('New lowest cost', min_cost)
                RRT_PATH = RRT_pot_path                                                                     # Updates RRT_PATH if a shorter path is found


    print('Optimal path coordinates: ', RRT_PATH)
    return RRT_PATH