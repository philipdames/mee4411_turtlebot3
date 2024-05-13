import rospy
from geometry_msgs.msg import Transform
import tf2_ros

import numpy as np
from typing import Optional, Tuple, Union


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
    # TODO fill x, y, theta in with correct values
    x = y = theta = 0.0
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
    # TODO fill in the transform
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
    # TODO fill in x, y, theta with correct values
    x = y = theta = 0.0
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
    # TODO fill in with the correct formula
    T = np.zeros((3,3))
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


def lookup_transform(tf_buffer: tf2_ros.Buffer, base_frame: str, child_frame: str, stamp: rospy.Time, time_travel: Optional[rospy.Duration]=rospy.Duration(0), format: Optional[str]='transform') -> Union[None, np.ndarray, Transform, Tuple]:
    '''
    Look up transformation on the TF2 buffer from base_frame to child_frame
    Inputs:
        tf_buffer: TF2 buffer object
        base_frame: base frame of transformation
        child_frame: child frame of transformation
        stamp: time stamp for TF lookup
        time_travel: amount to move backward in time
        format: format of the returned transformation 'transform', 'homogeneous', or 'xyt'
    Outputs:

    '''
    # Formation validation
    if not format in {'transform', 'homogeneous', 'xyt'}:
        rospy.logwarn(f'Format {format} not valid, must be: transform, homogeneous, or xyt')
        return None
    # Lookup transform
    try:
        trans = tf_buffer.lookup_transform(base_frame, child_frame, stamp - time_travel, rospy.Duration(2.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
        rospy.logwarn("TF lookup failed: ", error)
        return None
    # Return value
    if format == 'transform':
        return trans
    elif format == 'homogeneous':
        return transform2homogeneous(trans.transform)
    elif format == 'xyt':
        return transform2xyt(trans.transform)
