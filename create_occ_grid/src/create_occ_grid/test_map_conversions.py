#!/usr/bin/env python

import sys
import unittest
import numpy as np

import map_conversions as mc

## A sample python unit test
class TestMapConversions(unittest.TestCase):
    ## test sub2ind
    def test_sub2ind(self): # only functions with 'test_'-prefix will be run!
        # Set up test inputs
        rows = np.array([0, 0, 0, 1, 1, 1, 2, 1, -1])
        cols = np.array([0, 1, 2, 0, 1, 2, 2, 3, -2])
        shape = [2, 3]
        
        # Desired outputs
        inds_true = np.array([0, 1, 2, 3, 4, 5, -1, -1, -1])
        
        # Calculated outputs
        inds = mc.sub2ind(shape, rows, cols)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, rows.size):
            self.assertEquals(inds[i], inds_true[i], "sub (%d,%d) has the wrong index (ind %d instead of %d) in map of size %d by %d" % (rows[i], cols[i], inds[i], inds_true[i], shape[0], shape[1]))
    
    ## test ind2sub
    def test_ind2sub(self):
        # Set up test inputs
        inds = np.array([0, 1, 2, 3, 4, 5, -1, 8])
        shape = [3, 2]
        
        # Desired outputs
        rows_true = np.array([0, 0, 1, 1, 2, 2, -1, -1])
        cols_true = np.array([0, 1, 0, 1, 0, 1, -1, -1])
        
        # Calculated outputs
        rows, cols = mc.ind2sub(shape, inds)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, inds.size):
            self.assertEquals(rows[i], rows_true[i], "ind %d has the wrong row (%d instead of row %d) in map of size %d by %d" % (i, rows[i], rows_true[i], shape[0], shape[1]))
            self.assertEquals(cols[i], cols_true[i], "ind %d has the wrong column (%d instead of col %d) in map of size %d by %d" % (i, cols[i], cols_true[i], shape[0], shape[1]))
    
    ## test xy2sub
    def test_xy2sub(self):
        # Set up test inputs
        boundary = [-2., -2., 3., 3.]
        res = 1.
        x = np.array([-2., -1.5,  0.1, -0.6, 2., 3.,  0., -10., np.nan])
        y = np.array([-2., -1.5, -1.5, 1.25, 2., 3., 3.5, -10., np.nan])
        
        # Desired outputs
        rows_true = np.array([0, 0, 0, 3, 4, 4, -1, -1, -1])
        cols_true = np.array([0, 0, 2, 1, 4, 4, -1, -1, -1])
        
        # Calculated outputs
        rows, cols = mc.xy2sub(boundary, res, x, y)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, x.size):
            self.assertEquals(rows[i], rows_true[i], "(%.2f, %.2f) has wrong row (%d instead of %d) in map with boundary (%.2f, %.2f) to (%.2f, %.2f)" % (x[i], y[i], rows[i], rows_true[i], boundary[0], boundary[1], boundary[2], boundary[3]))
            self.assertEquals(cols[i], cols_true[i], "(%.2f, %.2f) has wrong column (%d instead of %d) in map with boundary (%.2f, %.2f) to (%.2f, %.2f)" % (x[i], y[i], cols[i], cols_true[i], boundary[0], boundary[1], boundary[2], boundary[3]))
    
    ## test sub2xy
    def test_sub2xy(self):
        # Set up test inputs
        boundary = [-2., -2., 3., 4.]
        res = 1.
        rows = np.array([0, 1, 0, 3, 5, 6])
        cols = np.array([0, 0, 2, 1, 4, 8])
        
        # Desired outputs
        x_true = np.array([-1.5, -1.5,  0.5, -0.5, 2.5, np.nan])
        y_true = np.array([-1.5, -0.5, -1.5,  1.5, 3.5, np.nan])
        
        # Calculated outputs
        x, y = mc.sub2xy(boundary, res, rows, cols)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, rows.size):
            x_err_str = "sub (%d, %d) has wrong x (%.2f instead of %.2f) in map with boundary (%.2f, %.2f) to (%.2f, %.2f)" % (rows[i], cols[i], x[i], x_true[i], boundary[0], boundary[1], boundary[2], boundary[3])
            y_err_str = "sub (%d, %d) has wrong y (%.2f instead of %.2f) in map with boundary (%.2f, %.2f) to (%.2f, %.2f)" % (rows[i], cols[i], y[i], y_true[i], boundary[0], boundary[1], boundary[2], boundary[3])
            if np.isnan(x_true[i]):
                self.assertTrue(np.isnan(x[i]), x_err_str)
                self.assertTrue(np.isnan(y[i]), y_err_str)
            else:
                self.assertEquals(x[i], x_true[i], x_err_str)
                self.assertEquals(y[i], y_true[i], y_err_str)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('mee4411_sandbox', 'test_map_conversions', TestMapConversions)

