import rospy
import tf2_ros

from .transform2d_utils import transform2homogeneous, transform2xyt

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
