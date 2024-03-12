#!/usr/bin/env python3

import rospy
import tf
import tf2_ros

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

import numpy as np
import threading
from typing import Tuple, Optional

        
class PurePursuit:
    def __init__(self):
        # initialize parameters
        self.lookahead   = rospy.get_param('~lookahead', 5.0) # lookahead distance [m]
        self.rate        = rospy.get_param('~rate', 10.) # rate to run controller [Hz]
        self.goal_margin = rospy.get_param('~goal_margin', 3.0) # maximum distance to goal before stopping [m]
            
        # Robot parameters
        robot_model = rospy.get_param('tb3_model', "")
        if robot_model == 'burger':
            self.wheel_seperation = 0.160 # [m]
            self.turning_radius   = 0.080 # [m]
            self.robot_radius     = 0.105 # [m]
            self.v_max            = 0.22 # maximum linear velocity [m/s]
            self.w_max            = 2.84 # maximum angular velocity [rad/s]
        elif robot_model == 'waffle' or robot_model == 'waffle_pi':
            self.wheel_seperation = 0.287 # [m]
            self.turning_radius   = 0.1435 # [m]
            self.robot_radius     = 0.220 # [m]
            self.v_max            = 0.26 # maximum linear velocity [m/s]
            self.w_max            = 1.82 # maximum angular velocity [rad/s]
        else:
            rospy.logerr('Turtlebot3 model %s not defined', robot_model)
        self.wheel_radius   = 0.033 # [m]
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'base_footprint')

        # Visualization markers
        # Initialize goal marker message
        self.goal_marker = Marker()
        self.goal_marker.ns = 'goal'
        self.goal_marker.type = Marker.SPHERE
        self.goal_marker.action = Marker.ADD
        self.goal_marker.pose.orientation.w = 1.0
        self.goal_marker.scale.x = 0.1
        self.goal_marker.scale.y = 0.1
        self.goal_marker.scale.z = 0.1
        self.goal_marker.color.r = 1.0
        self.goal_marker.color.a = 0.5 # set transparency

        # Circle marker
        self.circle_marker = Marker()
        self.circle_marker.header.frame_id = self.robot_frame_id
        self.circle_marker.ns = 'lookahead'
        self.circle_marker.type = Marker.LINE_STRIP
        self.circle_marker.action = Marker.ADD
        self.circle_marker.pose.orientation.w = 1.0
        self.circle_marker.scale.x = 0.03
        self.circle_marker.color.r = 1.0
        self.circle_marker.color.b = 1.0
        self.circle_marker.color.a = 0.5 # set transparency
        for theta in np.arange(0, 2*np.pi, np.pi/8):
            self.circle_marker.points.append(Point(self.lookahead*np.cos(theta), self.lookahead*np.sin(theta), 0))
        self.circle_marker.points.append(Point(self.lookahead, 0, 0))
    

    def findClosestPoint(self, path: Path, x: np.array, seg: Optional[int]=-1) -> Tuple[np.array, float, int]:
        '''
        Find the closest point on the current path to the point x
        Inputs: 
          path = nav_msgs/Path of the planned path
          x    = numpy array with 2 elements (x and y position of robot)
          seg  = optional argument that selects which segment of the path to compute the closest point on
        Outputs:
          pt_min   = closest point on the path to x
          dist_min = distance from the closest point to x
          seg_min  = index of closest segment to x
        '''
        # initialize return values
        pt_min = np.zeros(2) * np.nan
        dist_min = np.inf
        seg_min = -1
        
        ##### YOUR CODE STARTS HERE #####
        if seg == -1:
            # TODO find closest point on entire path
            pass # DELETE THIS LINE AND FILL IN YOUR CODE HERE
        else:
            # TODO find closest point on segment seg
            pass # DELETE THIS LINE AND FILL IN YOUR CODE HERE
            
        ##### YOUR CODE ENDS HERE #####
        return pt_min, dist_min, seg_min


    def findGoal(self, path: Path, x: np.array) -> np.array:
        '''
        Find the goal point to drive the robot towards
        Inputs: 
          path = nav_msgs/Path of the planned path
          x = numpy array with 2 elements (x and y position of robot)
        Outputs:
          goal = numpy array with 2 elements (x and y position of goal)
        '''
        # Find the closest point
        (pt, dist, seg) = self.findClosestPoint(path, x)
        if np.isnan(pt).any():
            raise Exception('No valid point found')

        # Find the goal
        goal = None
        if dist > self.lookahead:
            # if further than lookahead from the path, drive towards the path
            goal = pt
        else:
            ##### YOUR CODE STARTS HERE #####
            # TODO start from the nearest segment and iterate forward until you find either the last segment or a segment that leaves the lookahead circle
            pass
            
            # TODO if searched the whole path, set the goal as the end of the path
            pass
            
            # TODO if found a segment that leaves the circle, find the intersection with the circle
            pass
            ##### YOUR CODE ENDS HERE #####
            
        return goal
    

    def calculateVelocity(self, goal: np.array) -> Tuple[float, float]:
        '''
        Calculate the velocity of the robot. If the goal is closer than goal_margin, then set velocity to 0.
        Inputs: 
          goal = numpy array with 2 elements (x and y position of goal with respect to the robot)
        Outputs:
          v, w = linear and angular velocity of the robot
        '''
        # if goal is close enough to the robot, send 0 velocity
        if np.linalg.norm(goal) < self.goal_margin:
            v = 0.
            w = 0.
            return (v, w)

        ##### YOUR CODE STARTS HERE #####
        # TODO calculate the radius of curvature
        pass
        
        # TODO calculate the forward and angular velocity
        pass
        
        # TODO ensure velocity obeys the speed constraints
        v = 0. # DELETE THESE LINES AND FILL IN WITH YOUR CALCULATIONS
        w = 0.

        ##### YOUR CODE ENDS HERE #####
        return (v, w)
