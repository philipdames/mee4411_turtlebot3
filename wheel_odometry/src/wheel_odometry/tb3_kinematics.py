from sensor_msgs.msg import JointState

import numpy as np

# Indexing values
LEFT = 0
RIGHT = 1

X = 0
Y = 1
THETA = 2

def calculate_wheel_change(new_joint_states: JointState, prev_joint_states: JointState) -> list:
    # Inputs:
    #   new_joint_states    new joint states
    #   prev_joint_states   previous joint states
    # Outputs:
    #   delta_wheel_l       change in left wheel angle [rad]
    #   delta_wheel_r       change in right wheel angle [rad]
    #   delta_time          change in time [s]

    delta_wheel_l = 0 # FILL THIS IN
    delta_wheel_r = 0 # FILL THIS IN
    delta_time    = 0 # FILL THIS IN
    
    # Data validation
    if np.isnan(delta_wheel_l):
        delta_wheel_l = 0.0
    if np.isnan(delta_wheel_r):
        delta_wheel_r = 0.0

    return (delta_wheel_l, delta_wheel_r, delta_time)


def calculate_displacement(delta_wheel_l: float, delta_wheel_r: float, wheel_radius: float, wheel_separation: float) -> list:
    # Inputs:
    #   delta_wheel_l       change in left wheel angle [rad]
    #   delta_wheel_r       change in right wheel angle [rad]
    #   wheel_radius        wheel radius [m]
    #   wheel_separation    wheel separation [m]
    # Outputs:
    #   delta_s         linear displacement [m]
    #   delta_theta     angular displacement [rad]

    delta_s     = 0 # FILL THIS IN, linear displacement [m]
    delta_theta = 0 # FILL THIS IN, angular displacement [rad]

    return (delta_s, delta_theta)


def calculate_pose(prev_pose: list, delta_s: float, delta_theta: float) -> list:
    # Inputs:
    #   prev_pose       input pose in format (x, y, theta) [m, m, rad]
    #   delta_s         linear displacement [m]
    #   delta_theta     angular displacement [rad]
    # Outputs:
    #   pose            output pose in format (x, y, theta) [m, m, rad]

    pose = prev_pose # FILL THIS IN

    return pose
