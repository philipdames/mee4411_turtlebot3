#!/usr/bin/env python3
from geometry_msgs.msg import Transform

import unittest
import numpy as np

import transform2d_utils as t2d

## A sample python unit test
class TestTransform2DUtils(unittest.TestCase):
    ## test transform2xyt
    def test_transform2xyt(self): # only functions with 'test_'-prefix will be run!
        # Set up test inputs
        T = Transform()
        T.translation.x = 2.1
        T.translation.y = -1.3
        T.rotation.w = 0.722445
        T.rotation.z = 0.691428
        
        # Desired outputs
        x_true = 2.1
        y_true = -1.3
        theta_true = 1.526929
        pose_true = (x_true, y_true, theta_true)
        
        # Calculated outputs
        pose = t2d.transform2xyt(T)
        
        # Ensure that calculated outputs match desired outputs
        names = ['x', 'y', 'theta']
        for v, v_true, name in zip(pose, pose_true, names):
            self.assertTrue(np.abs(v - v_true) < 1e-4, f'{name} has the wrong value, {v:.6f} instead of {v_true:.6f}')
    

    ## test xyt2transform
    def test_xyt2transform(self):
        # Set up test inputs
        x = -2.9
        y = 2.3
        theta = -0.4
        
        # Desired outputs
        x = -2.9
        y = 2.3
        w = 0.980067
        z = -0.198669
        tf_true = (x, y, w, z)
        
        # Calculated outputs
        tf = t2d.xyt2transform(x, y, theta)
        tf = (tf.translation.x, tf.translation.y, tf.rotation.w, tf.rotation.z)
        
        # Ensure that calculated outputs match desired outputs
        names = ['t.x', 't.y', 'q.w', 'q.z']
        for v, v_true, name in zip(tf, tf_true, names):
            self.assertTrue(np.abs(v - v_true) < 1e-4, f'{name} has the wrong value, {v:.6f} instead of {v_true:.6f}')


    ## test homogeneous2xyt
    def test_homogeneous2xyt(self):
        # Set up test inputs
        c = 0.04385325885601188
        s = 0.9990379831056012
        x = 2.1
        y = -1.3
        T = np.array([[c, -s, x],
                      [s,  c, y],
                      [0., 0., 1.]])
        
        # Desired outputs
        x_true = x
        y_true = y
        theta_true = 1.526929
        pose_true = (x_true, y_true, theta_true)
        
        # Calculated outputs
        pose = t2d.homogeneous2xyt(T)
        
        # Ensure that calculated outputs match desired outputs
        names = ['x', 'y', 'theta']
        for v, v_true, name in zip(pose, pose_true, names):
            self.assertTrue(np.abs(v - v_true) < 1e-4, f'{name} has the wrong value, {v:.6f} instead of {v_true:.6f}')
    

    ## test xyt2homogeneous
    def test_xyt2homogeneous(self):
        # Set up test inputs
        x = 1.2
        y = -0.3
        theta = 2.65
        
        # Desired outputs
        T_true = np.array([[-0.8815821958782859, -0.47203054128988264, 1.2],
                           [0.47203054128988264,  -0.8815821958782859, -0.3],
                           [0.0, 0.0, 1.0]])
        
        # Calculated outputs
        T = t2d.xyt2homogeneous(x, y, theta)
        
        # Ensure that calculated outputs match desired outputs
        self.assertEquals(T.shape[0], 3, f'Homogeneous matrix has {T.shape[0]} rows instead of 3')
        self.assertEquals(T.shape[1], 3, f'Homogeneous matrix has {T.shape[1]} cols instead of 3')
        for r in range(2):
            for c in range(2):
                self.assertTrue(np.abs(T[r,c] - T_true[r,c]) < 1e-4, f'Element in ({r:d},{c:d}) has the wrong value, {T[r,c]:.6f} instead of {T_true[r,c]:.6f}')


if __name__ == '__main__':
    import rostest
    rostest.rosrun('create_occ_grid', 'test_transform2D_utils', TestTransform2DUtils)

