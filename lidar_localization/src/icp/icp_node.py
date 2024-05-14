import rospy
import tf2_ros

from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

import numpy as np
from threading import Lock

from .icp import MapICP
import transform2d_utils as t2d
from occupancy_grid import OccupancyGridMap

# Indexing values
X = 0
Y = 1
THETA = 2

class ICPLocalizationNode:
    def __init__(self) -> None:
        # Get initial pose (of odom with respect to map frame)
        initial_pose_x = rospy.get_param('~initial_pose_x', 0.0)
        initial_pose_y = rospy.get_param('~initial_pose_y', 0.0)
        initial_pose_a = rospy.get_param('~initial_pose_a', 0.0)

        # Store current pose (of odom with respect to map) in (x, y, theta) format
        self.pose = [initial_pose_x, initial_pose_y, initial_pose_a]

        # Initialize transformation (of odom with respect to map)
        self.tf_map_odom = TransformStamped()
        self.tf_map_odom.child_frame_id = rospy.get_param('~odom_frame', 'odom')

        # Initialize ICP object
        self.icp = MapICP()
        self.tolerance = None # tolerance for fitting
        self.lock = Lock() # to ensure ICP object can be locked

        # Get map
        use_map_topic = rospy.get_param('~use_map_topic', False)
        if use_map_topic:
            rospy.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback, queue_size=100)
            self.map = None
        else:
            rospy.wait_for_service('static_map')
            self.map_client = rospy.ServiceProxy('static_map', GetMap)
            rospy.sleep(0.1) # pause to let the service start up
            res = self.map_client()
            self.ogm = OccupancyGridMap(res.map)
            self.initalize_icp(res.map)

        # TF information
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_time_travel = rospy.Duration(rospy.get_param('~time_travel', 0.0)) # for shifting time, if needed

        # Publish initial transformation
        rospy.sleep(0.1) # pause to let the tf broadcaster start up
        self.publish_map_odom_tf(rospy.Time.now())

        # Scan subscriber
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)


    def initalize_icp(self, map: OccupancyGrid) -> None:
        # Set map tolerance to be the map cell size
        self.tolerance = map.info.resolution

        # Set transformation frame to be the map frame
        self.tf_map_odom.header.frame_id = map.header.frame_id

        ##### YOUR CODE STARTS HERE #####
        # TODO Extract the locations of all objects in the map
        pass

        # TODO use those points to inialize the ICP using the set_map_points method
        pass
        ##### YOUR CODE ENDS HERE   #####


    def map_callback(self, msg: OccupancyGrid) -> None:
        '''
        Save the map and use it to initialize the ICP map points
        '''
        with self.lock:
            self.ogm = OccupancyGridMap(msg)
            self.initalize_icp(msg)


    def publish_map_odom_tf(self, time: rospy.Time) -> None:
        '''
        Update the current transform from the map to odom frames stored in self.tf_map_odom
        '''
        ##### YOUR CODE STARTS HERE #####
        # TODO Fill in the current time
        pass

        # TODO Fill in the current transform (using the x, y, theta in self.pose)
        pass
        ##### YOUR CODE ENDS HERE   #####

        # Publish the transform
        self.tf_broadcaster.sendTransform(self.tf_map_odom)


    def scan_callback(self, msg: LaserScan) -> None:
        '''
        Process the lidar scan and use ICP to perform localization
        '''
        ##### YOUR CODE STARTS HERE #####
        # TODO Convert lidar all points to (x,y)
        # NOTE You should only keep points that are between the minimum and maximum range
        pass

        # TODO Lookup transformation from odom to lidar coordiante frame
        pass

        # TODO Use the transformation to transform the lidar points into the odom coordinate frame
        pass

        # TODO Use ICP to find pose of odom with respect to the map
        with self.lock:
            # NOTE need to do this inside of the lock to ensure correct operation
            pass

        # TODO Update self.pose using the transformation found by ICP
        pass
        ##### YOUR CODE ENDS HERE   #####

        # Publish transform
        self.publish_map_odom_tf(msg.header.stamp)
