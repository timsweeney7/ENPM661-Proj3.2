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
X_MAX = 600
Y_MAX = 250

# used accessing node information
PARENT_COORDINATES = 4
COORDINATES = 5


Open_List = []
Closed_List = []  # {"C2G", "C2C", "TC", "node_index", "parent_coor", "node_coor"}
Closed_Coor = set()
threshold_coor = set()
node_index = 0
obstacle_points = set()
map_points = set()



def draw_map():
    # Background
    background_color = BLACK
    map = np.zeros((250*SCALE_FACTOR, 600*SCALE_FACTOR, 3), np.uint8)
    map[:] = background_color

    # Map boarder
    map[0:ROBOT_SIZE*SCALE_FACTOR, :] = YELLOW                                    # north edge
    map[(Y_MAX - ROBOT_SIZE) * SCALE_FACTOR: Y_MAX * SCALE_FACTOR, :] = YELLOW    # south edge
    map[:, 0:ROBOT_SIZE * SCALE_FACTOR] = YELLOW                                   # east edge
    map[:, (X_MAX - ROBOT_SIZE) * SCALE_FACTOR: X_MAX * SCALE_FACTOR] = YELLOW      # west edge

    # box 1 boundary
    pts = np.array([[(100 - ROBOT_SIZE) * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [(150 + ROBOT_SIZE) * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [(150 + ROBOT_SIZE) * SCALE_FACTOR, (100 + ROBOT_SIZE) * SCALE_FACTOR],
                    [(100 - ROBOT_SIZE) * SCALE_FACTOR, (100 + ROBOT_SIZE) * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 1
    pts = np.array([[100 * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [150 * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [150 * SCALE_FACTOR, 100 * SCALE_FACTOR],
                    [100 * SCALE_FACTOR, 100 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # box 2  boundary
    pts = np.array([[(100 - ROBOT_SIZE) * SCALE_FACTOR, (150 - ROBOT_SIZE) * SCALE_FACTOR],
                    [(150 + ROBOT_SIZE) * SCALE_FACTOR, (150 - ROBOT_SIZE) * SCALE_FACTOR],
                    [(150 + ROBOT_SIZE) * SCALE_FACTOR, 250 * SCALE_FACTOR],
                    [(100 - ROBOT_SIZE) * SCALE_FACTOR, 250 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 2
    pts = np.array([[100*SCALE_FACTOR, 150*SCALE_FACTOR],
                    [150*SCALE_FACTOR, 150*SCALE_FACTOR],
                    [150*SCALE_FACTOR, 250*SCALE_FACTOR],
                    [100*SCALE_FACTOR, 250*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # hexagon boundry
    pts = np.array([[300*SCALE_FACTOR, (50 - ROBOT_SIZE) * SCALE_FACTOR],  # 1
                    [(365+ROBOT_SIZE)*SCALE_FACTOR, (87 - ROBOT_SIZE*tan(deg2rad(30)))*SCALE_FACTOR],  # 2
                    [(365+ROBOT_SIZE)*SCALE_FACTOR, (161 + ROBOT_SIZE*tan(deg2rad(30)))*SCALE_FACTOR],
                    [300*SCALE_FACTOR, (200 + ROBOT_SIZE) * SCALE_FACTOR],
                    [(235-ROBOT_SIZE)*SCALE_FACTOR, (161 + ROBOT_SIZE*tan(deg2rad(30)))*SCALE_FACTOR],
                    [(235-ROBOT_SIZE)*SCALE_FACTOR, (87 - ROBOT_SIZE*tan(deg2rad(30)))*SCALE_FACTOR]],  # 6
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # hexagon
    pts = np.array([[300*SCALE_FACTOR, 50*SCALE_FACTOR],
                    [365*SCALE_FACTOR, 87*SCALE_FACTOR],
                    [365*SCALE_FACTOR, 161*SCALE_FACTOR],
                    [300*SCALE_FACTOR, 200*SCALE_FACTOR],
                    [235*SCALE_FACTOR, 161*SCALE_FACTOR],
                    [235*SCALE_FACTOR, 87*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # triangle boundry
    pts = np.array([[(460-ROBOT_SIZE)*SCALE_FACTOR, (25-ROBOT_SIZE)*SCALE_FACTOR],
                    [(460+ROBOT_SIZE)*SCALE_FACTOR, (25-ROBOT_SIZE)*SCALE_FACTOR],
                    [(510+ROBOT_SIZE)*SCALE_FACTOR, 125*SCALE_FACTOR],
                    [(460+ROBOT_SIZE)*SCALE_FACTOR, (225+ROBOT_SIZE)*SCALE_FACTOR],
                    [(460-ROBOT_SIZE)*SCALE_FACTOR, (225+ROBOT_SIZE)*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # triangle
    pts = np.array([[460*SCALE_FACTOR, 25*SCALE_FACTOR],
                    [510*SCALE_FACTOR, 125*SCALE_FACTOR],
                    [460*SCALE_FACTOR, 225*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    return map


def get_valid_point_map(color_map):
    valid_point_map = np.ones((250 * SCALE_FACTOR, 600 * SCALE_FACTOR), np.uint8)
    for x in range(0, 600 * SCALE_FACTOR):
        for y in range(0, 250 * SCALE_FACTOR):
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
    if (x > 600) or (x < 0):
        return False
    elif (y > 250) or (y < 0):
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


#explore function that explores the next node located step-size away at +30 degrees and returns the new explored node
def explore_pos30 (n):
    angle = 30
    rad = math.radians(angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] + 30) % 360
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

#explore function that explores the next node located step-size away at +60 degrees and returns the new explored node
def explore_pos60 (n):
    angle = 60
    rad = math.radians(angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] + 60) % 360
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

#explore function that explores the next node located step-size away at -30 degrees and returns the new explored node
def explore_neg30 (n):
    angle = -30
    rad = math.radians(angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] - 30) % 360
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

#explore function that explores the next node located step-size away at -60 degrees and returns the new explored node
def explore_neg60 (n):
    angle = -60
    new_angle = n[5][2] + angle
    rad = math.radians(new_angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] - 60) % 360 
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

#explore function that explores the next node located step-size away straight ahead of the parent node and returns the new explored node
def explore_straight (n):
    angle = 0
    new_angle = n[5][2] + angle
    rad = math.radians(new_angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] + 0) % 360 
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[1]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

# main exploration function. starts by popping a node out of the open list and checking the status of the
# popped node prior to expanding the search using the exploration functions defined above.
# node: (C2G, C2C, TC, point_index, (x,y,theta)parent_coordinates, (x,y,theta)coordinates)
def exploreNodes(): 
    global goal_found
    hq.heapify(Open_List)
    while Open_List:
        if goal_found:
            break
        popped_node = hq.heappop(Open_List)
        Closed_Coor.add((popped_node[5][0], popped_node[5][1]))

#popped node is checked and added to the closed list as a dic
        check_popped_status(popped_node)
        popped_node_dic = {"C2G": popped_node[0], "C2C": popped_node[1], "TC": popped_node[2], "node_index": popped_node[3], "parent_coor": popped_node[4], "node_coor": popped_node[5]}
        Closed_List.append(popped_node_dic)

#checks if the newly created node falls within the defined map points, outside the obstacles, has not been closed already and its not within the threshhold of other points
#when all pass, it adds it to the open list 
        new_node = explore_pos30(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_pos60(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_neg30(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_pos60(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_straight(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        return Open_List, Closed_Coor, Closed_List
    
#threshhold function that checks if the newly created point falls within the already explored points, angle must be the same. point can have multiple different angle
def threshhold(nx, ny, nt):
    if (nx, ny, nt) in threshold_coor: 
        return True
    else: 
        threshold_coor.add((nx+0.5, ny, nt))
        threshold_coor.add((nx+0.5, ny+0.5, nt))
        threshold_coor.add((nx, ny+0.5, nt))
        threshold_coor.add((nx-0.5, ny+0.5, nt))
        threshold_coor.add((nx-0.5, ny, nt))
        threshold_coor.add((nx-0.5, ny-0.5, nt))
        threshold_coor.add((nx, ny-0.5, nt))
        threshold_coor.add((nx, ny-0.5, nt))
        return threshold_coor, False

#check if newly explored point has been explored previously, if so compare C2C and update if the new C2C is lower than the one originally stored
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
    if point_in_goal(n[5][0], n[5][1]) and n[5][2] == goal_position[2]:
        goal_node = {"C2G": n[0], "C2C": n[1], "TC": n[2], "node_index": n[3], "parent_coor": n[4], "node_coor": n[5]}
        Closed_List.append(goal_node)
        print("goal node position:", n[5])
        print("target position:", goal_position)
        print("Goal found")
        print("destination info:", n)
        goal_found = True
        start_backtrack ()
    else: 
        return(n)

#plotting function
def plot_function(path):
    closed_nodes = [node['node_coor'] for node in Closed_List]
    # plot_closed_nodes()
    # path_nodes = [node['node_coor'] for node in path]
    x_coords, y_coords = zip(*path)
    plt.scatter(*zip(*closed_nodes), marker='o', color='green', alpha=.1)
    plt.scatter(x_coords, y_coords, marker='o', color='black', alpha=1)
    # plt.plot(*zip(*path_nodes), color='black')

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
    
# (C2G, C2C, TC, point_index, (x,y,theta)parent_coordinates, (x,y,theta)coordinates)
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


    # user input required to fill out the following
    start_x_position = int(input("enter start X position(0-600): "))
    start_y_position = int(input("enter start Y position(0-250): "))
    start_theta_position = int(input("enter start theta position(0-360): "))
    print()
    goal_x_position = int(input("enter goal X position(0-600): "))
    goal_y_position = int(input("enter goal y position(0-250): "))
    goal_theta_position = int(input("enter goal theta position(0-360): "))
    print()
    ROBOT_SIZE = int(input("enter robot size (0-10): "))
    step_size = int(input("enter step size (0-10): "))

    start_position = (start_x_position, start_y_position, start_theta_position)
    goal_position = (goal_x_position, goal_y_position, goal_theta_position)

    clearance = 5
    thresh = 0.5

    # initial values for the start node
    C2G1 = C2G_func(start_position, goal_position)
    C2C1 = 0
    TC1 = C2G1 + C2C1
    # (C2G, C2C, TC, point_index, (x,y,theta)parent_coordinates, (x,y,theta)coordinates)
    start_node = (C2G1, C2C1, TC1, node_index, None, start_position)
    hq.heappush(Open_List, start_node)

    print("Drawing map...")
    color_map = draw_map()

    print("Determining open points and obstacle points...")
    valid_point_map = get_valid_point_map(color_map)
    # creat 2 sets, 1 containing all the possible points within map, 1 containg all possible points within the obstacle spaces
    # this will be used later to check the created points to see if they can be used
    x_range = np.arange(0, 600, 0.5)
    y_range = np.arange(0, 250, 0.5)
    for x in np.arange(0, 600, 0.5):
        for y in np.arange(0, 250, 0.5):
            if valid_point_map[int(y * SCALE_FACTOR), int(x * SCALE_FACTOR)] == 1:
                map_points.add((x, y))
            else:
                obstacle_points.add((x, y))
    test = len(map_points)+len(obstacle_points)

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
    
