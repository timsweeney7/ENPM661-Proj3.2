import cv2 as cv
import math
import numpy as np

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



"""
run_visualization()


"""
def run_visualization(color_map):
    
    cv.imshow('', color_map)

    cv.waitKey(0)
    cv.destroyAllWindows()
    return



if __name__ == "__main__":
    map = draw_map()
    run_visualization(map)