#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf

import numpy as np
import threading

class PurePursuit:
    # parameters of the controller
    lookahead = None # lookahead distance [m]
    rate = None # rate to run controller [Hz]
    goal_margin = None # maximum distance to goal before stopping [m]
    
    # parameters of the robot
    wheel_base = None # distance between left and right wheels [m]
    wheel_radius = None # wheel radius [m]
    v_max = None # maximum linear velocity [m/s]
    w_max = None # maximum angular velocity [rad/s]
    
    # ROS objects
    path_sub = None # subscriber to get the global path
    tf_listener = None # tf listener to get the pose of the robot
    cmd_vel_pub = None # publisher to send the velocity commands
    timer = None # timer to compute velocity commands
    
    # data
    path = None # store the path to the goal
    lock = threading.Lock() # lock to keep data thread safe
    
    # Constructor
    def __init__(self):
        # initialize parameters
        self.lookahead = rospy.get_param('~lookahead', 5.0)
        self.rate = rospy.get_param('~rate', 10.)
        self.goal_margin = rospy.get_param('~goal_margin', 3.0)
        
        self.wheel_base = rospy.get_param('~wheel_base', 0.16)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.033)
        self.v_max = rospy.get_param('~v_max', 0.22)
        self.w_max = rospy.get_param('~w_max', 2.84)
    
        # Initialize ROS objects
        self.path_sub = rospy.Subscriber('path', Path, self.path_callback)
        self.tf_listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Callback function for the path subscriber
    def path_callback(self, msg):
        rospy.logdebug('PurePursuit: Got path')
        # lock this data to ensure that it is not changed while other processes are using it
        self.lock.acquire()
        self.path = msg # store the path in the class member
        self.lock.release()
        # start the timer if this is the first path received
        if self.timer is None:
            self.start()
        
    # Start the timer that calculates command velocities
    def start(self):
        # initialize timer for controller update
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)
    
    # Get the current pose of the robot from the tf tree
    def get_current_pose(self):
        trans = rot = None
        # look up the current pose of the base_footprint using the tf tree
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Could not get robot pose')
            return (np.array([np.nan, np.nan]), np.nan)
        x = np.array([trans[0], trans[1]])
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(rot)
        rospy.logdebug("x = {}, y = {}, theta = {}".format(x[0], x[1], theta))
        
        return (x, theta)
    
    # Find the closest point on the current path to the point x
    # Inputs: 
    #   x = numpy array with 2 elements (x and y position of robot)
    #   seg = optional argument that selects which segment of the path to compute the closest point on
    # Outputs:
    #   pt_min = closest point on the path to x
    #   dist_min = distance from the closest point to x
    #   seg_min = index of closest segment to x
    def find_closest_point(self, x, seg=-1):
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
            # find closest point on entire path
            pass # DELETE THIS LINE AND FILL IN YOUR CODE HERE
        else:
            # find closest point on segment seg
            pass # DELETE THIS LINE AND FILL IN YOUR CODE HERE
            
        ##### YOUR CODE ENDS HERE #####
        return (pt_min, dist_min, seg_min)
    
    # Find the goal point to drive the robot towards
    # Inputs: 
    #   x = numpy array with 2 elements (x and y position of robot)
    #   pt, dist, seg = outputs of find_closest_point
    # Outputs:
    #   goal = numpy array with 2 elements (x and y position of goal)
    def find_goal(self, x, pt, dist, seg):
        goal = None
        if dist > self.lookahead:
            # if further than lookahead from the path, drive towards the path
            goal = pt
        else:
            ##### YOUR CODE STARTS HERE #####

            # start from the nearest segment and iterate forward until you find either the last segment or a segment that leaves the lookahead circle
            
            # if searched the whole path, set the goal as the end of the path
            
            # if found a segment that leaves the circle, find the intersection with the circle
            pass # DELETE THIS LINE AND FILL IN WITH YOUR CODE
            ##### YOUR CODE ENDS HERE #####
            
        return goal
    
    # Calculate the velocity of the robot. If the goal is closer than goal_margin, then set velocity to 0.
    # Inputs: 
    #   goal = numpy array with 2 elements (x and y position of goal with respect to the robot)
    # Outputs:
    #   v, w = linear and angular velocity of the robot
    def calculate_velocity(self, goal):
        # if goal is close enough to the robot, send 0 velocity
        if np.linalg.norm(goal) < self.goal_margin:
            v = 0.
            w = 0.
            return (v, w)

        ##### YOUR CODE STARTS HERE #####
           
        # calculate the radius of curvature
        
        # calculate the forward and angular velocity
        
        # ensure velocity obeys the speed constraints
        v = 0. # DELETE THESE LINES AND FILL IN WITH YOUR CALCULATIONS
        w = 0.

        ##### YOUR CODE ENDS HERE #####
        return (v, w)
    
    # function that runs every time the timer finishes to ensure that velocity commands are sent regularly
    def timer_callback(self, event):    
        # lock the path to ensure it is not updated during processing
        self.lock.acquire()
        try:
            # get current pose
            (x, theta) = self.get_current_pose()
            if np.isnan(x[0]): # ensure data is valid
                return
        
            # find the closest point
            (pt, dist, seg) = self.find_closest_point(x)
            if np.isnan(pt).any(): # ensure data is valid
                return
        
            # find the goal point
            goal = self.find_goal(x, pt, dist, seg)
            if goal is None: # ensure data is valid
                return
        finally:
            # ensure the lock is released
            self.lock.release()
        
        # transform goal to local coordinates
        ##### YOUR CODE STARTS HERE #####
        
        ##### YOUR CODE ENDS HERE #####
        
        # calculate the goal velocity of the robot and send the command
        cmd_vel = Twist()
        (cmd_vel.linear.x, cmd_vel.angular.z) = self.calculate_velocity(goal)
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z): # ensure data is valid
            self.cmd_vel_pub.publish(cmd_vel)
    

