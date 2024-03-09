#!/usr/bin/env python3

import sys
import unittest
import numpy as np

from pure_pursuit import PurePursuit

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


## A sample python unit test
class TestPurePursuit(unittest.TestCase):
    ## test find_closest_point
    def test_find_closest_point(self): # only functions with 'test_'-prefix will be run!
        # Set up path
        pp = PurePursuit()
        pp.path = Path()
        
        pose0 = PoseStamped()
        pose0.pose.position.x = 0.0
        pose0.pose.position.y = 0.0
        pp.path.poses.append(pose0)
        
        pose1 = PoseStamped()
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 0.0
        pp.path.poses.append(pose1)
        
        pose2 = PoseStamped()
        pose2.pose.position.x = 2.0
        pose2.pose.position.y = 1.0
        pp.path.poses.append(pose2)
        
        # Set up test inputs
        x = [np.array([-1., 0.]), np.array([0., 0.1]), np.array([0.5, 0.5]), np.array([0.9, -1.]), np.array([1.5, 0.5]), np.array([2., 0.5]), np.array([3., 3.])]
        #x = np.array([-1.,  0., 0.5, 0.9, 1.5, 2.0, 3.0])
        #y = np.array([ 0., 0.1, 0.5, -1., 0.5, 0.5, 3.0])
        
        # Desired outputs
        pts_true = [np.array([0., 0.]), np.array([0., 0.]), np.array([0.5, 0.]), np.array([0.9, 0.]), np.array([1.5, 0.5]), np.array([1.75, 0.75]), np.array([2., 1.])]
        dists_true = [1., 0.1, 0.5, 1.0, 0., 0.25*np.sqrt(2.), np.sqrt(5.)]
        segs_true = [0, 0, 0, 0, 1, 1, 1]
        
        # Ensure that calculated outputs match desired outputs
        err_msg_pt = "test point ({},{}) has the wrong closest point ({}, {}) instead of ({}, {})"
        err_msg_dist = "test point ({},{}) has the wrong distance ({} instead of {})"
        err_msg_seg = "test point ({},{}) has the wrong segment ({} instead of {})"
        for i in range(0, len(x)):
            (pt, dist, seg) = pp.findClosestPoint(x[i])
            self.assertTrue(np.linalg.norm(pt - pts_true[i]) < 1e-6, err_msg_pt.format(x[i][0], x[i][1], pt[0], pt[1], pts_true[i][0], pts_true[i][1]))
            self.assertTrue(np.abs(dist - dists_true[i]) < 1e-6, err_msg_dist.format(x[i][0], x[i][1], dist, dists_true[i]))
            self.assertEqual(seg, segs_true[i], err_msg_seg.format(x[i][0], x[i][1], seg, segs_true[i]))
    
    ## test find_goal
    def test_find_goal(self): # only functions with 'test_'-prefix will be run!
        # Set up path
        pp = PurePursuit()
        pp.path = Path()
        
        pose0 = PoseStamped()
        pose0.pose.position.x = 0.0
        pose0.pose.position.y = 0.0
        pp.path.poses.append(pose0)
        
        pose1 = PoseStamped()
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 0.0
        pp.path.poses.append(pose1)
        
        pose2 = PoseStamped()
        pose2.pose.position.x = 2.0
        pose2.pose.position.y = 1.0
        pp.path.poses.append(pose2)
        
        pp.lookahead = 1.0
        
        # Set up test inputs
        x = [np.array([-0.25, 0.0]), np.array([0.5, 0.]), np.array([1.5, 0.5])]
        
        # Desired outputs
        goals_true = [np.array([0.75, 0.]), np.array([1.41143782777, 0.411437827766]), np.array([2., 1.])]
        
        # Ensure that calculated outputs match desired outputs
        err_msg = "test point ({},{}) has the wrong goal ({}, {}) instead of ({}, {})"
        for i in range(0, len(x)):
            (pt, dist, seg) = pp.findClosestPoint(x[i])
            goal = pp.findGoal(x[i], pt, dist, seg)
            self.assertTrue(np.linalg.norm(goal - goals_true[i]) < 1e-6, err_msg.format(x[i][0], x[i][1], goal[0], goal[1], goals_true[i][0], goals_true[i][1]))
    
    ## test calculate_velocity
    def test_calculate_velocity(self): # only functions with 'test_'-prefix will be run!
        # Set up path
        pp = PurePursuit()
        
        pp.goal_margin = 0.1
        
        # Set up test inputs
        goals = [np.array([1., 0.]), np.array([1., 1.]), np.array([-0.5, -0.5]), np.array([0., -0.2]), np.array([0., 0.05])]
        
        # Desired outputs
        v_true = [0.22, 0.203703703704, -0.189655172414, 0.122222222222, 0.]
        w_true = [  0., 0.203703703704,  0.379310344828, -1.22222222222, 0.]
        
        # Ensure that calculated outputs match desired outputs
        err_msg = "test goal ({},{}) has the velocity ({}, {}) instead of ({}, {})"
        for i in range(0, len(goals)):
            (v, w) = pp.calculateVelocity(goals[i])
            self.assertTrue(np.abs(v - v_true[i]) < 1e-6 and np.abs(w - w_true[i]) < 1e-6, err_msg.format(goals[i][0], goals[i][1], v, w, v_true[i], w_true[i]))
    

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_pure_pursuit')
    rostest.rosrun('pure_pursuit', 'test_pure_pursuit', TestPurePursuit)

