#!/usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

import numpy as np
import threading

from pure_pursuit import PurePursuit
from transform2d_utils import lookup_transform


class PurePursuitNode(PurePursuit):

    # Constructor
    def __init__(self):
        # ROS parameters
        lookahead   = rospy.get_param('~lookahead', 5.0) # lookahead distance [m]
        goal_margin = rospy.get_param('~goal_margin', 3.0) # maximum distance to goal before stopping [m]
        robot_model = rospy.get_param('tb3_model', '')
        robot_frame_id = rospy.get_param('~robot_frame_id', 'base_footprint')

        # Initialize PurePursuit object
        super(PurePursuitNode, self).__init__(lookahead, goal_margin, robot_model, robot_frame_id)

        # Initialize ROS objects
        self.path_sub     = rospy.Subscriber('path', Path, self.path_callback) # subscriber to get the global path
        self.tfBuffer     = tf2_ros.Buffer()
        self.tfListener   = tf2_ros.TransformListener(self.tfBuffer) # tf listener to get the pose of the robot
        self.goal_vis_pub = rospy.Publisher('controller_marker', MarkerArray, queue_size=1, latch=True)
        self.cmd_vel_pub  = rospy.Publisher('cmd_vel', Twist, queue_size=10) # publisher to send the velocity commands
        self.timer = None # timer to compute velocity commands

        # Initialize data
        self.rate = rospy.get_param('~rate', 10.) # rate to run controller [Hz]
        self.path = None # store the path to the goal
        self.lock = threading.Lock() # lock to keep data thread safe
    

    def path_callback(self, msg: Path) -> None:
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
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)


    def timer_callback(self, event) -> None:
        '''
        Runs every time the timer finishes to ensure that velocity commands are sent regularly
        '''
        try:
            # Get current pose
            x, y, theta = lookup_transform(self.tfBuffer, self.path.header.frame_id, self.robot_frame_id, rospy.Time.now(), format='xyt')

            # Find the goal point
            with self.lock:
                goal = self.find_goal(self.path, np.array((x,y)))
            self.publish_pure_pursuit_markers(goal)
        except Exception as error:
            rospy.logerr(f'Cannot find goal: {error}')
            return

        ##### YOUR CODE STARTS HERE #####
        # TODO transform goal to local coordinates
        pass
        ##### YOUR CODE ENDS HERE #####

        # Calculate the goal velocity of the robot and send the command
        cmd_vel = Twist()
        (cmd_vel.linear.x, cmd_vel.angular.z) = self.calculate_velocity(goal)
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z): # ensure data is valid
            self.cmd_vel_pub.publish(cmd_vel)

        # Stop if near goal
        if np.linalg.norm(goal) < self.goal_margin:
            rospy.loginfo('PurePursuit: Reached goal')
            self.timer.shutdown()
            self.timer = None


    def publish_pure_pursuit_markers(self, goal) -> None:
        '''
        Publish markers showing the controller goal
        '''
        # Update markers
        self.update_markers(self.path.header.frame_id, goal, rospy.Time.now())

        # Create and publish marker array
        ma = MarkerArray()
        ma.markers.append(self.goal_marker)
        ma.markers.append(self.lookahead_marker)
        ma.markers.append(self.margin_marker)
        self.goal_vis_pub.publish(ma)    
