import math
import numpy as np

import map_conversions as mc

def create_occupancy_grid(boundary, blocks, res):
    # create_occupancy_grid creates an occupancy grid array representing the 
    # environment described by the inputs
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   blocks      array of arrays with [[xmin, ymin, xmax, ymax], ...] for each block
    #   res         cell size
    # outputs:
    #   numpy array containing the occupancy values in row-major order 
    #       Note: empty cells should have a value of 0 and occupied cells should
    #           have a value of 100
    #       Note: the lower left corner is the first cell
    #

    ##### YOUR CODE STARTS HERE #####
    
    ##### YOUR CODE ENDS HERE   #####

