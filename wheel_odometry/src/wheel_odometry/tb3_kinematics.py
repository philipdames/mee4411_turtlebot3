from sensor_msgs.msg import JointState

import numpy as np
from typing import List

from tb3_utils import TB3Params

# Indexing values
LEFT = 0
RIGHT = 1

X = 0
Y = 1
THETA = 2


class TB3Kinematics:
    def __init__(self, robot_model) -> None:
        self.params = TB3Params(robot_model)


    def calculate_wheel_change(self, new_joint_states: JointState, prev_joint_states: JointState) -> List[float]:
        # Inputs:
        #   new_joint_states    new joint states
        #   prev_joint_states   previous joint states
        # Outputs:
        #   delta_wheel_l       change in left wheel angle [rad]
        #   delta_wheel_r       change in right wheel angle [rad]
        #   delta_time          change in time [s]

        ##### YOUR CODE STARTS HERE #####
        # TODO Calculate the output values
        delta_wheel_l = 0.
        delta_wheel_r = 0.
        delta_time    = 0.
        ##### YOUR CODE ENDS HERE   #####
        
        # Data validation
        if np.isnan(delta_wheel_l):
            delta_wheel_l = 0.0
        if np.isnan(delta_wheel_r):
            delta_wheel_r = 0.0

        return (delta_wheel_l, delta_wheel_r, delta_time)


    def calculate_displacement(self, delta_wheel_l: float, delta_wheel_r: float) -> List[float]:
        # Inputs:
        #   delta_wheel_l       change in left wheel angle [rad]
        #   delta_wheel_r       change in right wheel angle [rad]
        # Outputs:
        #   delta_s         linear displacement [m]
        #   delta_theta     angular displacement [rad]

        ##### YOUR CODE STARTS HERE #####
        # TODO Calculate the output values
        delta_s     = 0. # linear displacement [m]
        delta_theta = 0. # angular displacement [rad]
        ##### YOUR CODE ENDS HERE   #####

        return (delta_s, delta_theta)


    def calculate_pose(self, prev_pose: List[float], delta_s: float, delta_theta: float) -> List[float]:
        # Inputs:
        #   prev_pose       input pose in format (x, y, theta) [m, m, rad]
        #   delta_s         linear displacement [m]
        #   delta_theta     angular displacement [rad]
        # Outputs:
        #   pose            output pose in format (x, y, theta) [m, m, rad]

        ##### YOUR CODE STARTS HERE #####
        # TODO Calculate the output values
        pose = prev_pose # FILL THIS IN
        ##### YOUR CODE ENDS HERE   #####

        return pose
