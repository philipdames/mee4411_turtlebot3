#!/usr/bin/env python3

import unittest
import numpy as np

import rospy
from sensor_msgs.msg import JointState

import tb3_kinematics as kn

## A sample python unit test
class TestKinematics(unittest.TestCase):
    ## test calculate_wheel_change
    def test_calculate_wheel_change(self): # only functions with 'test_'-prefix will be run!
        # Set up test inputs
        js_prev = JointState()
        js_prev.header.stamp = rospy.Time(0)
        js_prev.position = [0., 0.]
        
        js_new = JointState()
        js_new.header.stamp = js_prev.header.stamp + rospy.Duration(1.0)
        js_new.position = [0.5, -0.75]
        
        # Desired outputs
        delta_wheel_l_true = 0.5
        delta_wheel_r_true = -0.75
        delta_t_true = 1.0
        data_true = (delta_wheel_l_true, delta_wheel_r_true, delta_t_true)
        names = ['wheel left', 'wheel right', 'time']
        
        # Calculated outputs
        data = kn.calculate_wheel_change(js_new, js_prev)
        
        # Ensure that calculated outputs match desired outputs
        for t, d, n in zip(data_true, data, names):
            self.assertEqual(t, d, "Data %s has the wrong value (%.3f instead of %.3f)" % (n, d, t))
    
    
    ## test calculate_displacement
    def test_calculate_displacement(self):
        # Set up test inputs
        delta_wheel_l = np.array([0.696, -0.447, 0.676, 0.38, -0.4])
        delta_wheel_r = np.array([0.325, 0.083, -0.711, 0.629, 0.378])
        wheel_radius = np.array([0.113, 0.7, 0.585, 0.26, 0.231])
        wheel_separation = np.array([0.191, 0.331, 0.293, 0.559, 0.77])
        
        # Desired outputs
        delta_s_true = np.array([ 0.0576865, -0.1274, -0.0102375,  0.13117, -0.002541])
        delta_theta_true = np.array([-0.21949215,  1.12084592, -2.76926621,  0.11581395,  0.2334])
        
        # Calculated outputs
        delta_s = [None] * len(delta_wheel_l)
        delta_theta = [None] * len(delta_wheel_l)
        for i, (dwl, dwr, wr, ws) in enumerate(zip(delta_wheel_l, delta_wheel_r, wheel_radius, wheel_separation)):
            (delta_s[i], delta_theta[i]) = kn.calculate_displacement(dwl, dwr, wr, ws)
        
        # Ensure that calculated outputs match desired outputs
        for ds, dt, ds_true, dt_true in zip(delta_s, delta_theta, delta_s_true, delta_theta_true):
            self.assertTrue(np.abs(ds - ds_true) < 1e-6, "Linear displacement is wrong (%.6f instead of %.6f)" % (ds, ds_true))
            self.assertTrue(np.abs(dt - dt_true) < 1e-6, "Angular displacement is wrong (%.6f instead of %.6f)" % (dt, dt_true))
    

    ## test calculate_pose
    def test_calculate_pose(self):
        # Set up test inputs
        pose_in = [0.5, 0.3, -1.0]
        delta_s = 0.2
        delta_theta = 0.1
        
        # Desired outputs
        pose_true = [0.6163366178927767, 0.13731689904212524, -0.9]
        
        # Calculated outputs
        pose = kn.calculate_pose(pose_in, delta_s, delta_theta)
        
        # Ensure that calculated outputs match desired outputs
        names = ['x', 'y', 'theta']
        for p, p_true, n in zip(pose, pose_true, names):
            self.assertTrue(np.abs(p - p_true) < 1e-6, "Pose %s is wrong (%.6f instead of %.6f)" % (n, p, p_true))


if __name__ == '__main__':
    import rostest
    rostest.rosrun('wheel_odometry', 'test_kinematics', TestKinematics)
