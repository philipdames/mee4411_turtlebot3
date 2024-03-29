#!/usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

import numpy as np
from threading import Lock
from typing import Optional, Tuple, Union

import icp
import transform2D_utils as t2d
import map_conversions as mc

# Indexing values
X = 0
Y = 1
THETA = 2


class LidarLocalization:
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
        self.icp = icp.MapICP()
        self.tolerance = None # tolerance for fitting
        self.lock = Lock() # to ensure ICP object can be locked

        # Get map
        use_map_topic = rospy.get_param('~use_map_topic', False)
        if use_map_topic:
            rospy.map_sub = rospy.Subscriber('map', OccupancyGrid, self.mapCallback, queue_size=100)
            self.map = None
        else:
            rospy.wait_for_service('static_map')
            self.map_client = rospy.ServiceProxy('static_map', GetMap)
            rospy.sleep(0.1) # pause to let the service start up
            res = self.map_client()
            self.initalizeICP(res.map)

        # TF information
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_time_travel = rospy.Duration(rospy.get_param('~time_travel', 0.0)) # for shifting time, if needed

        # Publish initial transformation
        rospy.sleep(0.1) # pause to let the tf broadcaster start up
        self.publishMapOdomTF(rospy.Time.now())

        # Scan subscriber
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scanCallback, queue_size=1)


    def initalizeICP(self, map: OccupancyGrid) -> None:
        # Set map tolerance to be the map cell size
        self.tolerance = map.info.resolution

        # Set transformation frame to be the map frame
        self.tf_map_odom.header.frame_id = map.header.frame_id

        # Extract all the points in the map
        # TODO find the (x,y) locations of all objects in the map
        pass

        # TODO use those points to inialize the ICP using the setMapPoints method
        pass


    def mapCallback(self, msg: OccupancyGrid) -> None:
        with self.lock:
            self.initalizeICP(msg)
    

    def lookupTransform(self, base_frame: str, child_frame: str, stamp: rospy.Time, time_travel: Optional[rospy.Duration]=rospy.Duration(0)) -> Union[None, np.ndarray]:
        try:
            trans = self.tf_buffer.lookup_transform(base_frame, child_frame, stamp - time_travel, rospy.Duration(2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn("TF lookup failed: ", error)
            return None
        return t2d.transform2homogeneous(trans.transform)


    def publishMapOdomTF(self, time: rospy.Time) -> None:
        # Update the transform self.tf_map_odom
        # TODO fill in the current time
        pass

        # TODO fill in the current transform (using the x, y, theta in self.pose)
        pass

        # Publish the transform
        self.tf_broadcaster.sendTransform(self.tf_map_odom)


    def scanCallback(self, msg: LaserScan) -> None:
        # Convert lidar points to (x,y)
        # TODO use the data in the laser scan to find the (x,y) coordinate of each lidar hit in the lidar coordinate frame
        # NOTE you should only keep points that are between the minimum and maximum range
        pass

        # Lookup transformation from odom to lidar coordiante frame
        # TODO use the lookupTransform method to find this transformation
        pass

        # TODO use the transformation to transform the lidar points into the odom coordinate frame
        pass

        # Use ICP to find pose of odom with respect to the map
        with self.lock:
            # TODO call the ICP function to get the transformation
            pass

        # Update pose from the transformation
        # TODO use the transformation found from ICP to update self.pose

        # Publish transform
        self.publishMapOdomTF(msg.header.stamp)


if __name__ == '__main__':
    # Initialize objects
    rospy.init_node('lidar_localization')
    ll = LidarLocalization()
    rospy.spin()
