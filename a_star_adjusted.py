import heapq as hq
import copy
import numpy as np
import math
from matplotlib import pyplot as plt
import cv2 as cv
from numpy import tan, deg2rad



SCALE_FACTOR = 2

BLUE = (255, 0, 0)
DARK_GREEN = (15, 168, 33)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (9, 227, 212)
BLACK = (0, 0, 0)
GRAY = (199, 198, 195)

# for coordinates
X = 0
Y = 1

# map dimensions
X_MAX = 600 * SCALE_FACTOR
Y_MAX = 200 * SCALE_FACTOR

# used accessing node information
PARENT_COORDINATES = 4
COORDINATES = 5

# the turtlebot is 10.5 cm. 
# in an unscaled simulation, 1 pixel represents a 1cm x 1cm square
# mutliply ROBOT_SIZE by SCALE_FACTOR to determine pixel representation of robot
ROBOT_SIZE = 10.5 
ROBOT_SIZE = ROBOT_SIZE * SCALE_FACTOR
ROBOT_SIZE = math.ceil(ROBOT_SIZE)


Open_List = []    # used to track all the open nodes
Closed_List = []  # {"C2G", "C2C", "TC", "node_index", "parent_coor", "node_coor"} # used to track the closed node
Closed_Coor = set()   # closed coordinates. Used to quickly check if a coordinate is closed
threshold_coor = set()
obstacle_points = set()  # used to quickly look up if a point is in an obstacle
map_points = set()       # used to quickly look up if a point is in the map

node_index = 0


"""
draw_map()  
creates a (200 * SCALE_FACTOR) * (600 * SCALE_FACTOR) numpy array that represents the world map
the map represents a 2 meter by 6 meter world
using a SCALE_FACTOR of 2, each pixel represents a 50mm x 50mm square in the world
"""
def draw_map():
    # Background
    background_color = BLACK
    map = np.zeros((Y_MAX, X_MAX, 3), np.uint8)
    map[:] = background_color

    # Map boarder
    map[0:ROBOT_SIZE                , :                             ] = YELLOW    # north edge
    map[(Y_MAX - ROBOT_SIZE): Y_MAX , :                             ] = YELLOW    # south edge
    map[:                           , 0:ROBOT_SIZE                  ] = YELLOW    # east edge
    map[:                           , (X_MAX - ROBOT_SIZE) : X_MAX  ] = YELLOW    # west edge

    # box 1 boundary
    pts = np.array([[150 * SCALE_FACTOR - ROBOT_SIZE, 0 * SCALE_FACTOR],
                    [150 * SCALE_FACTOR - ROBOT_SIZE, 125 * SCALE_FACTOR + ROBOT_SIZE],
                    [165 * SCALE_FACTOR + ROBOT_SIZE, 125 * SCALE_FACTOR + ROBOT_SIZE],
                    [165 * SCALE_FACTOR + ROBOT_SIZE, 0 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 1
    pts = np.array([[150 * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [150 * SCALE_FACTOR, 125 * SCALE_FACTOR],
                    [165 * SCALE_FACTOR, 125 * SCALE_FACTOR],
                    [165 * SCALE_FACTOR, 0 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # box 2 boundary
    pts = np.array([[250 * SCALE_FACTOR - ROBOT_SIZE, 200 * SCALE_FACTOR],
                    [250 * SCALE_FACTOR - ROBOT_SIZE, 75 * SCALE_FACTOR - ROBOT_SIZE],
                    [265 * SCALE_FACTOR + ROBOT_SIZE, 75 * SCALE_FACTOR - ROBOT_SIZE],
                    [265 * SCALE_FACTOR + ROBOT_SIZE, 200 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 2
    pts = np.array([[250 * SCALE_FACTOR, 200 * SCALE_FACTOR],
                    [250 * SCALE_FACTOR, 75 * SCALE_FACTOR],
                    [265 * SCALE_FACTOR, 75 * SCALE_FACTOR],
                    [265 * SCALE_FACTOR, 200 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # circle boundry
    cv.circle(map, (400 * SCALE_FACTOR, 90 * SCALE_FACTOR), 50 * SCALE_FACTOR + ROBOT_SIZE, YELLOW, -1)

    # circle
    cv.circle(map, (400 * SCALE_FACTOR, 90 * SCALE_FACTOR), 50 * SCALE_FACTOR, BLUE, -1)

    return map


"""
get_valid_point_map()
input: np array representing the world map.  array values are the colors on the map
output: np array of 1s and 0s representing if a point is valid to be traveled in.
Function works by checking the color of pixel and determinining if that color is valid or invalid
"""
def get_valid_point_map(color_map):
    valid_point_map = np.ones((Y_MAX, X_MAX), np.uint8)
    for x in range(0, X_MAX):
        for y in range(0, Y_MAX):
            pixel_color = tuple(color_map[y, x])
            if pixel_color == YELLOW or pixel_color == BLUE:
                valid_point_map[y, x] = 0
    return valid_point_map


def determine_valid_point(valid_point_map, coordinates):
    if not __point_is_inside_map(coordinates[X], coordinates[Y]):
        return False
    if valid_point_map[coordinates[Y], coordinates[X]] == 1:
        return True
    else:
        return False


def __point_is_inside_map(x, y):
    if (x > X_MAX / SCALE_FACTOR) or (x < 0):
        return False
    elif (y > Y_MAX / SCALE_FACTOR) or (y < 0):
        return False
    else:
        return True

def __add_point(x, y, map, color):
    map[y, x] = color
    return map


def __draw_line(p1, p2, map, color):
    pts = np.array([[p1[0], p1[1]], [p2[0], p2[1]]],
                   np.int32)
    cv.fillPoly(map, [pts], color)


def draw_node(child_coordinates, parent_coordinates, map, color):

    child_coordinates = tuple(int(SCALE_FACTOR * _) for _ in child_coordinates)
    cv.circle(map, child_coordinates, radius=3, color=color, thickness=-1)

    if (parent_coordinates is not None):
        parent_coordinates = tuple(int(SCALE_FACTOR * _ ) for _ in parent_coordinates)
        cv.circle(map, parent_coordinates, radius=3, color=color, thickness=-1)
        __draw_line(child_coordinates, parent_coordinates, map, color)


#function that defines the goal position as a circle with a threshhold.
# takes in the size of the robot as the threshhold for the target
def point_in_goal(x, y):
    distance = math.sqrt((x-goal_position[0])**2 + (y-goal_position[1])**2)
    if distance <= ROBOT_SIZE:
        return True
    else:
        return False


#function to calculate the cost to go to from the point to the goal in a straight line
def C2G_func (n_position, g_position): 
    C2G = round(((g_position[0]-n_position[0])**2 + (g_position[1]-n_position[1])**2)**0.5, 1)
    return C2G

RPM_1 = 2
RPM_2 = 12
r = 1
L = 4
dt = .5

actions = [[RPM_1, 0], [RPM_1, RPM_1], [0, RPM_1], [RPM_2, 0], [RPM_2, RPM_2], [0, RPM_2], [RPM_1, RPM_2], [RPM_2, RPM_1]]

def explore(n,UL,UR):
    t = 0
    D=0
    Xn=n[5][0]
    Yn=n[5][1]
    Thetan = 3.14 * n[5][2] / 180
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    while t<1:
        Xs = Xn
        Ys = Yn
        Thetan += (r / L) * (UR - UL) * dt
        Xn += round(((0.5*r * (UL + UR) * math.cos(Thetan) * dt)/2)*2)
        Yn += round(((0.5*r * (UL + UR) * math.sin(Thetan) * dt)/2)*2)
        # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
        t = t + dt
    
    Thetan = 180 * (Thetan) / 3.14
    new_position = round(Xn,2), round(Yn,2), round(Thetan,2)
    D=round(D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2)), 2)
    new_C2C = n[1]+D
    new_C2G = C2G_func(new_position, goal_position)
    new_TC = new_C2C + new_C2G
    new_node = (new_TC, new_C2C, new_C2G, node_index, n[5], new_position)
    return new_node

# main exploration function. starts by popping a node out of the open list and checking the status of the
# popped node prior to expanding the search using the exploration functions defined above.
# node: (C2G, C2C, TC, point_index, (x,y,theta)parent_coordinates, (x,y,theta)coordinates)
def exploreNodes(): 
    global goal_found
    hq.heapify(Open_List)
    # print("Open_List_init", Open_List)
    while Open_List:
        if goal_found:
            break
        popped_node = hq.heappop(Open_List)
        # print("starting node:", popped_node)
        Closed_Coor.add((popped_node[5][0], popped_node[5][1]))
        # print("closed coor:", Closed_Coor)
        # print(map_points)

#popped node is checked and added to the closed list as a dic
        check_popped_status(popped_node)
        popped_node_dic = {"TC": popped_node[0], "C2C": popped_node[1], "C2G": popped_node[2], "node_index": popped_node[3], "parent_coor": popped_node[4], "node_coor": popped_node[5]}
        Closed_List.append(popped_node_dic)
        # print("Closed List:", Closed_List)

#checks if the newly created node falls within the defined map points, outside the obstacles, has not been closed already and its not within the threshhold of other points
#when all pass, it adds it to the open list 
        new_node = explore(copy.deepcopy(popped_node), actions[0][0], actions[0][1])
        # print("NEW1:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points:
            # print("First Check") 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                # print("Second Check") 
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # print("Third Check") 
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore(copy.deepcopy(popped_node), actions[1][0], actions[1][1])
        # print("NEW2:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore(copy.deepcopy(popped_node), actions[2][0], actions[2][1])
        # print("NEW3:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore(copy.deepcopy(popped_node), actions[3][0], actions[3][1])
        # print("NEW4:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore(copy.deepcopy(popped_node), actions[4][0], actions[4][1])
        # print("NEW5:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore(copy.deepcopy(popped_node), actions[5][0], actions[5][1])
        # print("NEW6:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore(copy.deepcopy(popped_node), actions[6][0], actions[6][1])
        # print("NEW7:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore(copy.deepcopy(popped_node), actions[7][0], actions[7][1])
        # print("NEW8:", new_node)
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    # if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                    checkC2C(copy.deepcopy(popped_node), new_node)
        # print("OpenList:", Open_List)
        return Open_List, Closed_Coor, Closed_List
    
#threshhold function that checks if the newly created point falls within the already explored points,
# angle must be the same. point can have multiple different angle
def threshhold(nx, ny):
    if (nx, ny) in threshold_coor: 
        return True
    else: 
        threshold_coor.add((nx+0.5, ny))
        threshold_coor.add((nx+0.5, ny+0.5))
        threshold_coor.add((nx, ny+0.5))
        threshold_coor.add((nx-0.5, ny+0.5))
        threshold_coor.add((nx-0.5, ny))
        threshold_coor.add((nx-0.5, ny-0.5))
        threshold_coor.add((nx, ny-0.5))
        threshold_coor.add((nx, ny-0.5))
        return threshold_coor, False

#check if newly explored point has been explored previously, if so compare C2C and update if the new C2C is
# lower than the one originally stored
def checkC2C (on, n):
    global node_index
    node_index += 1
    new = (n[0], n[1], n[2], node_index, on[5], n[5])
    hq.heappush(Open_List, new)
    hq.heapify(Open_List)
    return Open_List

#function to check the status of the popped node, if it matches the goal coordinates it starts the backtracking function
def check_popped_status (n):
    global goal_found
    if point_in_goal(n[5][0], n[5][1]):
        goal_node = {"TC": n[0], "C2C": n[1], "C2G": n[2], "node_index": n[3], "parent_coor": n[4], "node_coor": n[5]}
        Closed_List.append(goal_node)
        print("goal node position:", n[5])
        print("target position:", goal_position)
        print("Goal found")
        print("destination info:", n)
        goal_found = True
        start_backtrack ()
    else: 
        return(n)


#backtracking function that takes in the last closed node from the closed list and runs a loop using
#  the node coordinates and the parent coordinate to trace back the steps leading to the start position
def start_backtrack ():
    print("Backtracking...")
    path_nodes = []
    path_coor = []
    current_node = Closed_List[-1]
    path_nodes.append(current_node)
    path_coor.append((current_node['node_coor'][0], current_node['node_coor'][1]))
    print("First node used:", current_node)
    
# (TC, C2C, C2G, point_index, (x,y,theta)parent_coordinates, (x,y,theta)coordinates)
    while current_node["parent_coor"] is not None:
        search_value = current_node["parent_coor"]
        for node in Closed_List:
            if node["node_coor"] == search_value:
                # If a matching value is found, assign the entire dictionary as the new current_node
                current_node = node
                break
        path_nodes.append(current_node)
        path_coor.append((current_node['node_coor'][0], current_node['node_coor'][1]))
    #print(path_coor)

    path_coor.reverse()
    run_visualization(path_coor)
    #plot_function(path_coor)
    #plt.show()
    #print("length of closed list:", len(Closed_List))
    #print("length of closed path:", len(path_nodes))
    #print("length of closed path coor:", len(path_coor))


def run_visualization(path_coordinates):
    counter = 0
    for node in Closed_List:
        child_coordinates = (node["node_coor"][X], node["node_coor"][Y])
        if node["parent_coor"] == None:
            parent_coordinates = None
        else:
            parent_coordinates = (node["parent_coor"][X], node["parent_coor"][Y])
        draw_node(child_coordinates, parent_coordinates, color_map, GRAY)
        counter += 1
        if counter == 50:
            cv.imshow('Djikstra\'s Algorith', color_map)
            cv.waitKey(1)
            counter = 0

    parent = None
    counter += 1
    for node in path_coordinates:
        draw_node(node, parent, color_map, DARK_GREEN)
        parent = node
        cv.imshow('Djikstra\'s Algorith', color_map)
        cv.waitKey(1)

    node = path_coordinates[-1]
    draw_node(node, parent, color_map, RED)
    cv.imshow('Djikstra\'s Algorith', color_map)

    cv.waitKey(0)
    cv.destroyAllWindows()
    return


if __name__ == '__main__':

    DEBUG = True

    # user input required to fill out the following
    if DEBUG == False:
        start_x_position = int(input("enter start X position(0-600): "))
        start_y_position = int(input("enter start Y position(0-250): "))
        start_theta_position = int(input("enter start theta position(0-360): "))
        print()
        goal_x_position = int(input("enter goal X position(0-600): "))
        goal_y_position = int(input("enter goal y position(0-250): "))
        print()
        RPM_1 = int(input("enter RPM for wheel rotation speed 1 (0-100): "))
        RPM_2 = int(input("enter RPM for wheel rotation speed 2 (0-100): "))
        print()
        step_size = int(input("enter step size (0-10): "))
        clearance = int(input("enter the clearance used for navigation (in mm): "))
        ROBOT_SIZE = ROBOT_SIZE + clearance
    else:
        start_x_position  = 50
        start_y_position  = 180
        start_theta_position  = 0
        goal_x_position  = 540
        goal_y_position  = 180
        # RPM_1 = 50
        # RPM_2 = 100
        # step_size = 1
        ROBOT_SIZE = 5  # (mm) 105 is the size if the robot in mm.  Will not work for this simulation
        clearance = 2
        ROBOT_SIZE = ROBOT_SIZE + clearance




    start_position = (start_x_position, start_y_position, start_theta_position)
    goal_position = (goal_x_position, goal_y_position)

    thresh = 0.5

    # initial values for the start node
    C2G1 = C2G_func(start_position, goal_position)
    C2C1 = 0
    TC1 = C2G1 + C2C1
    # (C2G, C2C, TC, point_index, (x,y,theta)parent_coordinates, (x,y,theta)coordinates)
    start_node = (TC1, C2C1, C2G1, node_index, None, start_position)
    hq.heappush(Open_List, start_node)
    print("initial_Open_list:", Open_List)

    print("Drawing map...")
    color_map = draw_map()

    print("Determining open points and obstacle points...")
    valid_point_map = get_valid_point_map(color_map)
    # creat 2 sets, 1 containing all the possible points within map, 1 containg all possible points within the obstacle spaces
    # this will be used later to check the created points to see if they can be used
    x_range = np.arange(0, 600, 0.5)
    y_range = np.arange(0, 200, 0.5)
    for x in np.arange(0, 600, 0.5):
        for y in np.arange(0, 200, 0.5):
            if valid_point_map[int(y * SCALE_FACTOR), int(x * SCALE_FACTOR)] == 1:
                map_points.add((x, y))
            else:
                obstacle_points.add((x, y))

    if (start_position[0], start_position[1]) in obstacle_points:
        print("start point selected is in obstacle space, try again")
        exit()
    if (start_position[0], start_position[1]) not in map_points:
        print("start point selected is  outside the map, try again")
        exit()
    if (goal_position[0], goal_position[1]) in obstacle_points:
        print(goal_position)
        print("goal point selected is in obstacle space, try again")
        exit()
    if (goal_position[0], goal_position[1]) not in map_points:
        print("goal point selected is  outside the map, try again")
        exit()

    goal_found = False
    print("start node:", start_node)
    print("starting exploration")
    while not goal_found:
        exploreNodes()
        # print("OpenList1:", Open_List)