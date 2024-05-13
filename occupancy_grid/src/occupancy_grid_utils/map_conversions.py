import math
import numpy as np

class OccupancyGridMap:
    def __init__(self) -> None:
        pass

def sub2ind(array_shape, rows, cols):
    # sub2ind coverts subscript (row, column) pairs into linear indices in
    # row-major order
    #
    # inputs:
    #   array_shape array with [# of rows, # of columns]
    #   rows        numpy array of row indices
    #   cols        numpy array of column indices
    # outputs:
    #   numpy array of integer indices
    #       Note: (row, column) pairs that are not valid should have a 
    #       corresponding output index of -1
    #
    
    ##### YOUR CODE STARTS HERE #####
    pass
    ##### YOUR CODE ENDS HERE   #####

def ind2sub(array_shape, ind):
    # ind2sub converts linear indices in a row-major array to subscript
    # (row, column) pairs
    #
    # inputs:
    #   array_shape array with [# of rows, # of columns]
    #   ind         numpy array of integer indices
    # outputs:
    #   numpy array of row indices
    #   numpy array of column indices
    #       Note: any indices that are not valid should have row and column
    #       subscripts outputs of -1
    #
    
    ##### YOUR CODE STARTS HERE #####
    pass
    ##### YOUR CODE ENDS HERE   #####
    
def xy2sub(boundary, res, x, y):
    # xy2sub converts (x,y) coordinate pairs into (row, column) subscript pairs
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   x           numpy array of x values
    #   y           numpy array of y values
    # outputs:
    #   numpy array of row indices
    #   numpy array of column indices
    #       Note: any (x,y) pairs that are not valid should have subscript
    #       outputs of -1
    #
    
    ##### YOUR CODE STARTS HERE #####
    pass
    ##### YOUR CODE ENDS HERE   #####
    
def sub2xy(boundary, res, rows, cols):
    # sub2xy converts (row, column) subscript pairs into (x,y) coordinate pairs
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   rows        numpy array of row indices
    #   cols        numpy array of column indices
    # outputs:
    #   numpy array of x coordinates of center of each cell
    #   numpy array of y coordinates of center of each cell
    #       Note: any (row, col) pairs that are not valid should have outputs
    #       of numpy NaN
    #
    
    ##### YOUR CODE STARTS HERE #####
    pass
    ##### YOUR CODE ENDS HERE   #####

def xy2ind(boundary, res, array_shape, x, y):
    # xy2ind converts (x,y) coordinate pairs into linear indices in row-major
    # order
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   array_shape numpy array with [# of rows, # of columns]
    #   x           numpy array of x values
    #   y           numpy array of y values
    # outputs:
    #   numpy array of row indices
    #   numpy array of column indices
    #
    
    rows, cols = xy2sub(boundary, res, x, y)
    ind = sub2ind(array_shape, rows, cols)
    return ind

def ind2xy(boundary, res, array_shape, ind):
    # ind2xy converts linear indices in row-major order into (x,y) coordinate
    # pairs
    #
    # inputs:
    #   boundary    array with [xmin, ymin, xmax, ymax]
    #   res         cell size
    #   array_shape numpy array with [# of rows, # of columns]
    #   ind         numpy array of indices
    # outputs:
    #   numpy array of x coordinates
    #   numpy array of y coordinates
    #
    
    rows, cols = ind2sub(array_shape, ind)
    x, y = sub2xy(boundary, res, rows, cols)
    return (x, y)

