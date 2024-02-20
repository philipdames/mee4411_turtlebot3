#!/usr/bin/env python3

import rospy
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import numpy as np

import tb3_kinematics as kn

# Indexing values
LEFT = 0
RIGHT = 1

X = 0
Y = 1
THETA = 2


class WheelOdometry:
    def __init__(self) -> None:
        # Robot parameters
        robot_model = rospy.get_param('tb3_model', "")
        if robot_model == 'burger':
            self.wheel_separation = 0.160 # [m]
            self.turning_radius = 0.080 # [m]
            self.robot_radius = 0.105 # [m]
        elif robot_model == 'waffle' or robot_model == 'waffle_pi':
            self.wheel_separation = 0.287 # [m]
            self.turning_radius = 0.1435 # [m]
            self.robot_radius = 0.220 # [m]
        else:
            rospy.logerr('Turtlebot3 model %s not defined', robot_model)
        self.wheel_radius = 0.033 # [m]

        # Joint states data
        self.prev_joint_states = None

        # Odometry data
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
        self.joint_states_sub = rospy.Subscriber('joint_states', JointState, self.jointStatesCallback, queue_size=100)
    

    def jointStatesCallback(self, msg: JointState) -> None:
        # Check if first message received
        if self.prev_joint_states is None:
            self.prev_joint_states = msg
            return

        # Update and send messages
        self.updateOdometry(msg)
        self.updateTF()

        # Save message
        self.prev_joint_states = msg


    def updateOdometry(self, new_joint_states: JointState) -> None:
        # Calculate change in wheel angles
        # (TO DO: use new msg and self.prev_joint_states)
        (delta_wheel_l, delta_wheel_r, delta_time) = kn.calculate_wheel_change()

        # Calculate displacement 
        # (TO DO: use changes in wheel angles and robot kinematics)
        (delta_s, delta_theta) = kn.calculate_displacement()

        # Compute new pose
        # (TO DO: update self.pose based on displacement)
        self.pose = kn.calculate_pose()
        # FILL THIS IN

        # Update odometry (stored in self.odom)
        # (TO DO: fill in self.odom_.header based on current time)
        # FILL THIS IN

        # (TO DO: fill in self.odom_.pose.pose based on calculated pose values)
        # FILL THIS IN

        # (TO DO: fill in self.odom_.twist.twist based on calculated displacements and time)
        # FILL THIS IN

        # Publish odometry
        self.odom_pub_.publish(self.odom)
    
    
    def updateTF(self) -> None:
        # Initialize empty transform
        odom_tf = TransformStamped()

        # Fill in transform from odom_frame to base_frame
        # (TO DO: fill in transform using data calculated from self.odom_)
        # FILL THIS IN

        # Broadcast transform
        self.tf_broadcaster.sendTransform(odom_tf)


if __name__ == '__main__':
    # Initialize objects
    rospy.init_node('wheel_odometry')
    wo = WheelOdometry()
    rospy.spin()
