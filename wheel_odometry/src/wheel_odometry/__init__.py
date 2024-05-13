#!/usr/bin/env python3

import rospy
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import numpy as np

from .tb3_kinematics import TB3Kinematics

# Indexing values
LEFT = 0
RIGHT = 1

X = 0
Y = 1
THETA = 2


class WheelOdometryNode(TB3Kinematics):
    def __init__(self) -> None:
        # Robot parameters
        robot_model = rospy.get_param('tb3_model', "")
        if not robot_model in ('burger', 'waffle', 'waffle_pi'):
            rospy.logerr(f'Turtlebot3 model {robot_model} not defined')
        super(WheelOdometryNode, self).__init__(robot_model)

        # Joint states data
        self.prev_joint_states = None

        # Odometry message
        self.odom = Odometry()
        self.odom.header.frame_id = rospy.get_param('odom_frame', 'odom')
        self.odom.child_frame_id = rospy.get_param('base_frame', 'base_footprint')
        self.odom.pose.covariance = np.diag((0.1, 0.1, 1e6, 1e6, 1e6, 0.2)).flatten().tolist()
        self.odom.twist.covariance = np.diag((0.1, 0.1, 1e6, 1e6, 1e6, 0.2)).flatten().tolist()
        self.pose = [0.0, 0.0, 0.0] # (x, y, theta)

        # Publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=100)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribers
        self.joint_states_sub = rospy.Subscriber('joint_states', JointState, self.joint_states_callback, queue_size=100)


    def joint_states_callback(self, msg: JointState) -> None:
        # Check if first message received
        if self.prev_joint_states is None:
            self.prev_joint_states = msg
            return

        # Update and send messages
        self.update_odometry(msg)
        self.update_TF()

        # Save message for future use
        self.prev_joint_states = msg


    def update_odometry(self, new_joint_states: JointState) -> None:
        '''
        Update and publish the odometry message based on the new and previous joint states
        '''
        ##### YOUR CODE STARTS HERE #####
        # TODO Calculate change in wheel angles
        # NOTE Use new msg and self.prev_joint_states
        pass

        # TODO Calculate displacement
        pass

        # TODO Compute new pose
        pass

        # TODO Update odometry message (stored in self.odom) based on new time, pose, and velocity
        pass
        ##### YOUR CODE ENDS HERE   #####

        # Publish odometry
        self.odom_pub_.publish(self.odom)
    
    
    def update_TF(self) -> None:
        '''
        Update and publish the transformation from the odom frame to the base frame of the robot
        '''
        # Initialize empty transform
        odom_tf = TransformStamped()

        ##### YOUR CODE STARTS HERE #####
        # TODO Fill in transform from odom_frame to base_frame
        pass
        ##### YOUR CODE ENDS HERE   #####

        # Broadcast transform
        self.tf_broadcaster.sendTransform(odom_tf)
