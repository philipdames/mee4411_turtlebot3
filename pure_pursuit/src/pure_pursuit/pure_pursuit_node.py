#!/usr/bin/env python3

import rospy
import tf
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

import numpy as np
import threading
from typing import Tuple, Optional

from pure_pursuit import PurePursuit


class PurePursuitNode(PurePursuit):
    
    # Constructor
    def __init__(self):
        # Initialize PurePursuit object
        super(PurePursuitNode, self).__init__()

        # Initialize ROS objects
        self.path_sub     = rospy.Subscriber('path', Path, self.pathCallback) # subscriber to get the global path
        self.tfBuffer     = tf2_ros.Buffer()
        self.tfListener   = tf2_ros.TransformListener(self.tfBuffer) # tf listener to get the pose of the robot
        self.goal_vis_pub = rospy.Publisher('~goal_marker', MarkerArray, queue_size=1, latch=True)
        self.cmd_vel_pub  = rospy.Publisher('cmd_vel', Twist, queue_size=10) # publisher to send the velocity commands
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
            with self.lock:
                trans, rot = self.tfBuffer.lookupTransform(self.path.header.frame_id, self.robot_frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        x = np.array([trans[0], trans[1]])
        _, _, theta = tf.transformations.euler_from_quaternion(rot)
        rospy.logdebug("x = {}, y = {}, theta = {}".format(x[0], x[1], theta))
        
        return (x, theta)


    def timerCallback(self, event) -> None:
        '''
        Runs every time the timer finishes to ensure that velocity commands are sent regularly
        '''
        try:
            # Get current pose
            (x, theta) = self.getCurrentPose()

            # Find the goal point
            with self.lock:
                goal = self.findGoal(self.path, x)
            self.publishPurePursuitMarkers(goal)
        except Exception as error:
            rospy.logerr(f'Cannot find goal: {error}')
            return
        
        ##### YOUR CODE STARTS HERE #####
        # TODO transform goal to local coordinates
        pass
        ##### YOUR CODE ENDS HERE #####
        
        # Calculate the goal velocity of the robot and send the command
        cmd_vel = Twist()
        (cmd_vel.linear.x, cmd_vel.angular.z) = self.calculateVelocity(goal)
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z): # ensure data is valid
            self.cmd_vel_pub.publish(cmd_vel)

        # Stop if near goal
        if np.linalg.norm(goal) < self.goal_margin:
            self.timer.shutdown()
            self.timer = None


if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    
    pp = PurePursuitNode()
    
    rospy.spin()

