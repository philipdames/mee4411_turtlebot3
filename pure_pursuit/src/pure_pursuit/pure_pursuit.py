#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf
import tf2_ros

import numpy as np
import threading
from typing import Tuple, Optional


class PurePursuit:
    
    # Constructor
    def __init__(self):
        # Initialize controller parameters
        self.lookahead   = rospy.get_param('~lookahead', 5.0) # lookahead distance [m]
        self.rate        = rospy.get_param('~rate', 10.) # rate to run controller [Hz]
        self.goal_margin = rospy.get_param('~goal_margin', 3.0) # maximum distance to goal before stopping [m]

        # Robot parameters
        robot_model = rospy.get_param('tb3_model', 'burger')
        if robot_model == 'burger':
            self.wheel_separation = 0.160 # [m]
            self.turning_radius   = 0.080 # [m]
            self.robot_radius     = 0.105 # [m]
        elif robot_model == 'waffle' or robot_model == 'waffle_pi':
            self.wheel_separation = 0.287 # [m]
            self.turning_radius   = 0.1435 # [m]
            self.robot_radius     = 0.220 # [m]
        else:
            rospy.logerr('Turtlebot3 model %s not defined', robot_model)
        self.wheel_radius = 0.033 # [m]
        self.v_max = rospy.get_param('~v_max', 0.22) # maximum linear velocity [m/s]
        self.w_max = rospy.get_param('~w_max', 2.84) # maximum angular velocity [rad/s]
        
        # Initialize ROS objects
        self.path_sub    = rospy.Subscriber('path', Path, self.pathCallback) # subscriber to get the global path
        self.tfBuffer    = tf2_ros.Buffer()
        self.tfListener  = tf2_ros.TransformListener(self.tfBuffer) # tf listener to get the pose of the robot
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) # publisher to send the velocity commands
        timer = None # timer to compute velocity commands
        
        # Initialize data
        path = None # store the path to the goal
        lock = threading.Lock() # lock to keep data thread safe
    

    def pathCallback(self, msg: Path) -> None:
        '''
        Callback function for the path subscriber
        '''
        rospy.logdebug('PurePursuit: Got path')
        # lock this data to ensure that it is not changed while other processes are using it
        with self.lock:
            self.path = msg # store the path in the class member
        # start the timer if this is the first path received
        if self.timer is None:
            self.start()
        

    def start(self) -> None:
        '''
        Start the timer that calculates command velocities
        '''
        # initialize timer for controller update
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timerCallback)
    

    def getCurrentPose(self):
        '''
        Get the current pose of the robot from the tf tree
        '''
        try:
            trans, rot = self.tfBuffer.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('Could not get robot pose')
            return (np.array([np.nan, np.nan]), np.nan)
        x = np.array([trans[0], trans[1]])
        _, _, theta = tf.transformations.euler_from_quaternion(rot)
        rospy.logdebug("x = {}, y = {}, theta = {}".format(x[0], x[1], theta))
        
        return (x, theta)
    

    def findClosestPoint(self, x: np.array, seg: Optional[int]=-1) -> Tuple[np.array, float, int]:
        '''
        Find the closest point on the current path to the point x
        Inputs: 
          x   = numpy array with 2 elements (x and y position of robot)
          seg = optional argument that selects which segment of the path to compute the closest point on
        Outputs:
          pt_min   = closest point on the path to x
          dist_min = distance from the closest point to x
          seg_min  = index of closest segment to x
        '''
        # initialize return values
        pt_min = np.array([np.nan, np.nan])
        dist_min = np.inf
        seg_min = -1
        
        # check if path has been received yet
        if self.path is None:
            rospy.logwarn('Pure Pursuit: No path received yet')
            return (pt_min, dist_min, seg_min)
        
        ##### YOUR CODE STARTS HERE #####
        if seg == -1:
            # TODO find closest point on entire path
            pass # DELETE THIS LINE AND FILL IN YOUR CODE HERE
        else:
            # TODO find closest point on segment seg
            pass # DELETE THIS LINE AND FILL IN YOUR CODE HERE
            
        ##### YOUR CODE ENDS HERE #####
        return (pt_min, dist_min, seg_min)


    def findGoal(self, x: np.array, pt: np.array, dist: float, seg: int) -> np.array:
        '''
        Find the goal point to drive the robot towards
        Inputs: 
          x = numpy array with 2 elements (x and y position of robot)
          pt, dist, seg = outputs of find_closest_point
        Outputs:
          goal = numpy array with 2 elements (x and y position of goal)
        '''
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
    

    def timerCallback(self, event) -> None:
        '''
        Runs every time the timer finishes to ensure that velocity commands are sent regularly
        '''
        # lock the path to ensure it is not updated during processing
        with self.lock:
            try:
                # get current pose
                (x, theta) = self.getCurrentPose()
                if np.isnan(x[0]): # ensure data is valid
                    return
            
                # find the closest point
                (pt, dist, seg) = self.findClosestPoint(x)
                if np.isnan(pt).any(): # ensure data is valid
                    return
            
                # find the goal point
                goal = self.findGoal(x, pt, dist, seg)
                if goal is None: # ensure data is valid
                    return
            except:
                raise
        
        ##### YOUR CODE STARTS HERE #####
        # TODO transform goal to local coordinates
        pass
        ##### YOUR CODE ENDS HERE #####
        
        # calculate the goal velocity of the robot and send the command
        cmd_vel = Twist()
        (cmd_vel.linear.x, cmd_vel.angular.z) = self.calculateVelocity(goal)
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z): # ensure data is valid
            self.cmd_vel_pub.publish(cmd_vel)
