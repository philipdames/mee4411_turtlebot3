import math
import numpy as np

class MapConversions:
    def __init__(self, boundary, resolution: float) -> None:
        self.boundary = boundary
        self.resolution = resolution
        ##### YOUR CODE STARTS HERE #####
        # Create the array shape
        self.array_shape = None
        ##### YOUR CODE ENDS HERE   #####

    def sub2ind(self, rows, cols):
        '''
        sub2ind coverts subscript (row, column) pairs into linear indices in
        row-major order
        
        inputs:
            rows    numpy array of row indices
            cols    numpy array of column indices
        outputs:
            inds    numpy array of integer indices
        Note: (row, column) pairs that are not valid should have a 
              corresponding output index of -1
        
        '''
        ##### YOUR CODE STARTS HERE #####
        inds = -1 * np.ones_like(rows)
        ##### YOUR CODE ENDS HERE   #####
        return inds

    def ind2sub(self, inds):
        '''
        ind2sub converts linear indices in a row-major array to subscript
        (row, column) pairs
        
        inputs:
            inds    numpy array of integer indices
        outputs:
            rows    numpy array of row indices
            cols    numpy array of column indices
        Note: any indices that are not valid should have row and column
              subscripts outputs of -1
        '''
        ##### YOUR CODE STARTS HERE #####
        rows = -1 * np.ones_like(inds)
        cols = -1 * np.ones_like(inds)
        ##### YOUR CODE ENDS HERE   #####
        return rows, cols
    
    def xy2sub(self, x, y):
        '''
        xy2sub converts (x,y) coordinate pairs into (row, column) subscript pairs
        
        inputs:
            x       numpy array of x values
            y       numpy array of y values
        outputs:
            rows    numpy array of row indices
            cols    numpy array of column indices
        Note: any (x,y) pairs that are not valid should have subscript
              outputs of -1
        '''
        ##### YOUR CODE STARTS HERE #####
        rows = -1 * np.ones_like(x)
        cols = -1 * np.ones_like(y)
        ##### YOUR CODE ENDS HERE   #####
        return rows, cols
    
    def sub2xy(self, rows, cols):
        '''
        sub2xy converts (row, column) subscript pairs into (x,y) coordinate pairs
        
        inputs:
            rows        numpy array of row indices
            cols        numpy array of column indices
        outputs:
            x       numpy array of x coordinates of center of each cell
            y       numpy array of y coordinates of center of each cell
        Note: any (row, col) pairs that are not valid should have outputs
              of numpy NaN
        '''
        ##### YOUR CODE STARTS HERE #####
        x = np.nan * np.ones_like(rows)
        y = np.nan * np.ones_like(cols)
        ##### YOUR CODE ENDS HERE   #####
        return x, y

    def xy2ind(self, x, y):
        '''
        xy2ind converts (x,y) coordinate pairs into linear indices in row-major
        order
        
        inputs:
            x           numpy array of x values
            y           numpy array of y values
        outputs:
            numpy array of row indices
            numpy array of column indices
        '''
        rows, cols = self.xy2sub(x, y)
        ind = self.sub2ind(rows, cols)
        return ind

    def ind2xy(self, ind):
        '''
        ind2xy converts linear indices in row-major order into (x,y) coordinate
        pairs
        
        inputs:
            ind         numpy array of indices
        outputs:
            numpy array of x coordinates
            numpy array of y coordinates
        '''
        rows, cols = self.ind2sub(ind)
        x, y = self.sub2xy(rows, cols)
        return (x, y)


class OccupancyGridMap:
    def __init__(self) -> None:
        pass