#!/usr/bin/env python3

import unittest
import numpy as np

from map_conversions import MapConversions

## A sample python unit test
class TestMapConversions(unittest.TestCase):
    ## test sub2ind
    def test_sub2ind(self): # only functions with 'test_'-prefix will be run!
        # Set up MapConversions
        boundary = [0., 0., 3., 2.]
        resolution = 1.
        mc = MapConversions(boundary, resolution)

        # Set up test inputs
        rows = np.array([0, 0, 0, 1, 1, 1, 2, 1, -1])
        cols = np.array([0, 1, 2, 0, 1, 2, 2, 3, -2])
        shape = [2, 3]
        
        # Desired outputs
        inds_true = np.array([0, 1, 2, 3, 4, 5, -1, -1, -1])
        
        # Calculated outputs
        inds = mc.sub2ind(rows, cols)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, rows.size):
            self.assertEquals(inds[i], inds_true[i], f"sub ({rows[i]}, {cols[i]}) has the wrong index (ind {inds[i]} instead of {inds_true[i]}) in map of size {shape[0]} by {shape[1]}")


    ## test ind2sub
    def test_ind2sub(self):
        # Set up MapConversions
        boundary = [0., 0., 2., 3.]
        resolution = 1.
        mc = MapConversions(boundary, resolution)

        # Set up test inputs
        inds = np.array([0, 1, 2, 3, 4, 5, -1, 8])
        shape = [3, 2]
        
        # Desired outputs
        rows_true = np.array([0, 0, 1, 1, 2, 2, -1, -1])
        cols_true = np.array([0, 1, 0, 1, 0, 1, -1, -1])
        
        # Calculated outputs
        rows, cols = mc.ind2sub(inds)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, inds.size):
            self.assertEquals(rows[i], rows_true[i], f"ind {i} has the wrong row ({rows[i]} instead of row {rows_true[i]}) in map of size {shape[0]} by {shape[1]}")
            self.assertEquals(cols[i], cols_true[i], f"ind {i} has the wrong column ({cols[i]} instead of col {cols_true[i]}) in map of size {shape[0]} by {shape[1]}")


    ## test xy2sub
    def test_xy2sub(self):
        # Set up MapConversions
        boundary = [-2., -2., 3., 3.]
        resolution = 1.
        mc = MapConversions(boundary, resolution)

        # Set up test inputs
        x = np.array([-2., -1.5,  0.1, -0.6, 2., 3.,  0., -10., np.nan])
        y = np.array([-2., -1.5, -1.5, 1.25, 2., 3., 3.5, -10., np.nan])
        
        # Desired outputs
        rows_true = np.array([0, 0, 0, 3, 4, 4, -1, -1, -1])
        cols_true = np.array([0, 0, 2, 1, 4, 4, -1, -1, -1])
        
        # Calculated outputs
        rows, cols = mc.xy2sub(x, y)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, x.size):
            self.assertEquals(rows[i], rows_true[i], f"point ({x[i]:.2f}, {y[i]:.2f}) has wrong row ({rows[i]} instead of row {rows_true[i]}) in map with boundary ({boundary[0]:.2f}, {boundary[1]:.2f}) to ({boundary[2]:.2f}, {boundary[3]:.2f})")
            self.assertEquals(cols[i], cols_true[i], f"point ({x[i]:.2f}, {y[i]:.2f}) has wrong column ({cols[i]} instead of col {cols_true[i]})) in map with boundary ({boundary[0]:.2f}, {boundary[1]:.2f}) to ({boundary[2]:.2f}, {boundary[3]:.2f})")


    ## test sub2xy
    def test_sub2xy(self):
        # Set up MapConversions
        boundary = [-2., -2., 3., 4.]
        resolution = 1.
        mc = MapConversions(boundary, resolution)
        
        # Set up test inputs
        rows = np.array([0, 1, 0, 3, 5, 6])
        cols = np.array([0, 0, 2, 1, 4, 8])
        
        # Desired outputs
        x_true = np.array([-1.5, -1.5,  0.5, -0.5, 2.5, np.nan])
        y_true = np.array([-1.5, -0.5, -1.5,  1.5, 3.5, np.nan])
        
        # Calculated outputs
        x, y = mc.sub2xy(rows, cols)
        
        # Ensure that calculated outputs match desired outputs
        for i in range(0, rows.size):
            x_err_str = f"sub ({rows[i]}, {cols[i]}) has wrong x ({x[i]:.2f} instead of {x_true[i]:.2f}) in map with boundary ({boundary[0]:.2f}, {boundary[1]:.2f}) to ({boundary[2]:.2f}, {boundary[3]:.2f})"
            y_err_str = f"sub ({rows[i]}, {cols[i]}) has wrong y ({y[i]:.2f} instead of {y_true[i]:.2f}) in map with boundary ({boundary[0]:.2f}, {boundary[1]:.2f}) to ({boundary[2]:.2f}, {boundary[3]:.2f})"
            if np.isnan(x_true[i]):
                self.assertTrue(np.isnan(x[i]), x_err_str)
                self.assertTrue(np.isnan(y[i]), y_err_str)
            else:
                self.assertEquals(x[i], x_true[i], x_err_str)
                self.assertEquals(y[i], y_true[i], y_err_str)
