from nav_msgs.msg import OccupancyGrid

import numpy as np
from typing import Optional, Tuple, Union

class MapConversions:
    def __init__(self, boundary, resolution: float) -> None:
        # Boundary of the envrionment in the format (xmin, ymin, xmax, ymax)
        self.boundary = boundary
        # Size of the cells in the occupancy grid
        self.resolution = resolution
        ##### YOUR CODE STARTS HERE #####
        # TODO Create the array shape in the format (# rows, # columns)
        self.array_shape = (0, 0)
        ##### YOUR CODE ENDS HERE   #####


    @classmethod
    def from_occupancy_grid_msg(cls, msg: OccupancyGrid):
        '''
        Create an object from an OccupancyGrid ROS msg
        '''
        ##### YOUR CODE STARTS HERE #####
        # Extract the boundary and cell resolution from the occupancy grid message
        boundary = [0, 0, 1, 1]
        resolution = 1.
        ##### YOUR CODE ENDS HERE   #####
        return cls(boundary, resolution)


    def sub2ind(self, rows: np.array, cols: np.array):
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
        # TODO Convert data in (row, col) format to ind format
        inds = -np.ones_like(rows)
        ##### YOUR CODE ENDS HERE   #####
        return inds


    def ind2sub(self, inds: np.array):
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
        # TODO Convert data in ind format to (row, col) format
        rows = -np.ones_like(inds)
        cols = -np.ones_like(inds)
        ##### YOUR CODE ENDS HERE   #####
        return rows, cols


    def xy2sub(self, x: np.array, y: np.array):
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
        # TODO Convert data in (x, y) format to (row, col) format
        rows = -np.ones_like(x)
        cols = -np.ones_like(y)
        ##### YOUR CODE ENDS HERE   #####
        return rows, cols


    def sub2xy(self, rows: np.array, cols: np.array):
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
        # TODO Convert data in (row, col) format to (x, y) format
        x = np.nan * np.ones_like(rows)
        y = np.nan * np.ones_like(cols)
        ##### YOUR CODE ENDS HERE   #####
        return x, y


    def xy2ind(self, x: np.array, y: np.array):
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


    def ind2xy(self, inds: np.array):
        '''
        ind2xy converts linear indices in row-major order into (x,y) coordinate
        pairs
        
        inputs:
            inds        numpy array of indices
        outputs:
            numpy array of x coordinates
            numpy array of y coordinates
        '''
        rows, cols = self.ind2sub(inds)
        x, y = self.sub2xy(rows, cols)
        return (x, y)


class OccupancyGridMap(MapConversions):
    def __init__(self, boundary, resolution, frame_id) -> None:
        super(OccupancyGridMap, self).__init__(boundary, resolution)
        # Set coordinate frame ID
        self.frame_id = frame_id
        # Initialize empty data array (2D array holding values)
        #   In the range [0, 1], representing the probability of occupancy
        #   If a cell is unknown, set to -1
        self.data = np.zeros(self.array_shape)


    @classmethod
    def from_occupancy_grid_msg(cls, msg: OccupancyGrid):
        '''
        Create an object from an OccupancyGrid msg
        '''
        # Initialize object
        ogm = super(OccupancyGridMap, cls).from_occupancy_grid_msg(msg)
        ##### YOUR CODE STARTS HERE #####
        # TODO Update data array, based on conventions above
        pass
        ##### YOUR CODE ENDS HERE   #####
        return ogm


    def add_block(self, block: np.array) -> None:
        '''
        Add a block to the map stored in self.data
        Inputs:
            block   np.array in the format (xmin, ymin, xmax, ymax)
        '''
        ##### YOUR CODE STARTS HERE #####
        # TODO Fill in all the cells that overlap with the block
        pass
        ##### YOUR CODE ENDS HERE   #####


    def to_occupancy_grid_msg(self) -> OccupancyGrid:
        '''
        Convert the OccupancyGridMap object into an OccupancyGrid ROS message
        Inputs:
            None
        Outputs:
            msg     OccupancyGrid ROS message
        '''
        msg = OccupancyGrid()
        ##### YOUR CODE STARTS HERE #####
        # TODO Fill in all the fields of the msg using the data from the class
        pass
        ##### YOUR CODE ENDS HERE   #####
        return msg


    def is_occupied(self, x: np.array, y: np.array) -> bool:
        '''
        Check whether the given cells are occupied
        Inputs:
            x           numpy array of x values
            y           numpy array of y values
        Outputs:
            occupied    np.array of bool values
        '''
        ##### YOUR CODE STARTS HERE #####
        # TODO Check for occupancy in the map based on the input type
        occupied = np.zeros_like(x, dtype='bool')
        ##### YOUR CODE ENDS HERE   #####
        return occupied


    def where_occupied(self, format='xy') -> np.array:
        '''
        Find the locations of all cells that are occupied
        Inputs:
            format      requested format of the returned data ('xy', 'rc', 'ind')
        Outputs:
            locations   np.array with the locations of occupied cells in the requested format
        '''
        # Check that requested format is valid
        if not format in ('xy', 'rc', 'ind'):
            raise Exception(f'Requested format {format} invalid, must be xy, rc, or ind')
        ##### YOUR CODE STARTS HERE #####
        # TODO Check for occupancy in the map based on the input type
        locations = np.zeros(2)
        ##### YOUR CODE ENDS HERE   #####
        return locations
