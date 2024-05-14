from geometry_msgs.msg import Transform

import numpy as np
from typing import Tuple


def transform2xyt(T: Transform) -> Tuple[float, float, float]:
    '''
    Convert geometry_msgs/Transform to (x, y, theta)
    Inputs:
        T: geometry_msgs/Transform object
    Outputs:
        x: position [m]
        y: position [m]
        theta: angle [rad]
    '''
    ##### YOUR CODE STARTS HERE #####
    # TODO fill x, y, theta in with correct values
    x = y = theta = 0.0
    ##### YOUR CODE ENDS HERE #####
    return (x, y, theta)


def xyt2transform(x: float, y: float, theta: float) -> Transform:
    '''
    Convert (x, y, theta) to geometry_msgs/Transform
    Inputs:
        x: position [m]
        y: position [m]
        theta: angle [rad]
    Outputs:
        T: geometry_msgs/Transform object
    '''
    T = Transform()
    ##### YOUR CODE STARTS HERE #####
    # TODO fill in the transform
    pass
    ##### YOUR CODE ENDS HERE #####
    return T


def homogeneous2xyt(T: np.ndarray) -> Tuple[float, float, float]:
    '''
    Convert homogeneous transformation matrix to (x, y, theta)
    Inputs:
        T: 3x3 numpy.array of the transformation matrix
    Outputs:
        x: position [m]
        y: position [m]
        theta: angle [rad]
    '''
    ##### YOUR CODE STARTS HERE #####
    # TODO fill in x, y, theta with correct values
    x = y = theta = 0.0
    ##### YOUR CODE ENDS HERE #####
    return (x, y, theta)


def xyt2homogeneous(x: float, y: float, theta: float) -> Tuple[np.ndarray]:
    '''
    Convert (x, y, theta) to homogeneous transformation
    Inputs:
        x: position [m]
        y: position [m]
        theta: angle [rad]
    Outputs:
        T: 3x3 numpy.array of the transformation matrix
    '''
    ##### YOUR CODE STARTS HERE #####
    # TODO fill in with the correct formula
    T = np.zeros((3,3))
    ##### YOUR CODE ENDS HERE #####
    return T


def transform2homogeneous(T: Transform) -> np.ndarray:
    '''
    Convert geometry_msgs/Transform to homogeneous transformation
    Inputs:
        T: geometry_msgs/Transform object
    Outputs:
        T: 3x3 numpy.array of the transformation matrix
    '''
    return xyt2homogeneous(*transform2xyt(T))


def homogeneous2transform(T: np.ndarray) -> Transform:
    '''
    Convert homogeneous transformation to geometry_msgs/Transform
    Inputs:
        T: 3x3 numpy.array of the transformation matrix
    Outputs:
        T: geometry_msgs/Transform object
    '''
    return xyt2transform(*homogeneous2xyt(T))
