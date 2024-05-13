#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import LaserScan, JointState

from copy import deepcopy
import numpy as np


class SimRepub:
    def __init__(self) -> None:
        # Namespace to remove from frames
        self.namespace_remove_ = rospy.get_param('~namespace_remove', '')
        self.noise_std_ = rospy.get_param('~noise_std', 0.0)
        self.alpha_ = rospy.get_param('~alpha', 0.9) # running average param
        # ROS publishers and subscribers
        self.scan_pub_ = rospy.Publisher('~scan_out', LaserScan, queue_size=10)
        self.joint_states_pub_ = rospy.Publisher('~joint_states_out', JointState, queue_size=10)
        self.prev_joint_state_ = None # previous joint states
        self.joint_state_rate_ = None # rate at which messages are being published
        self.position_ = [0.0, 0.0]
        self.velocity_ = [0.0, 0.0]
        rospy.sleep(0.1)
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scanCallback)
        self.joint_states_sub_ = rospy.Subscriber('joint_states', JointState, self.jointStateCallback)
    

    def stripName(self, name: str) -> str:
        # Strip "self.namespace_remove_" from "name"
        parts = name.split('/')
        parts = [p for p in parts if p not in ['', self.namespace_remove_] ]
        return "/".join(parts)


    def scanCallback(self, msg: LaserScan) -> None:
        msg.header.frame_id = self.stripName(msg.header.frame_id)
        self.scan_pub_.publish(msg)
    

    def jointStateCallback(self, msg: JointState) -> None:
        msg_out = deepcopy(msg)
        # Change names
        msg_out.header.frame_id = self.stripName(msg.header.frame_id)
        msg_out.name = [self.stripName(n) for n in msg.name]
        # Add noise (only if moving)
        if self.noise_std_ > 0.0 and any([abs(v) > 0 for v in msg.velocity]):
            if not self.prev_joint_state_ is None:
                # Get timing information
                dt = (msg.header.stamp - self.prev_joint_state_.header.stamp).to_sec()
                if self.joint_state_rate_ is None:
                    self.joint_state_rate_ = dt
                else:
                    self.joint_state_rate_ = self.alpha_ * self.joint_state_rate_ + (1-self.alpha_) * dt
                # Get change in position
                delta = [p - q + np.random.randn() * self.noise_std_ * self.joint_state_rate_ for p, q in zip(msg.position, self.prev_joint_state_.position)]
                self.position_ = [p + d for p, d in zip(self.position_, delta)]
                msg_out.position = self.position_
                msg_out.velocity = [v + np.random.randn() * self.noise_std_ * self.joint_state_rate_ for v in msg.velocity]
            self.prev_joint_state_ = msg
        # Publish message
        self.joint_states_pub_.publish(msg_out)


if __name__ == '__main__':
    rospy.init_node('sim_repub')
    sr = SimRepub()
    rospy.spin()
