#!/usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetMap

import cv2 as cv
import numpy as np
from threading import Lock
from typing import Optional, Union

from prm import PRM
import transform2D_utils as t2d


class GlobalPlanner:
    def __init__(self) -> None:
        # Robot parameters
        robot_model = rospy.get_param('tb3_model', "", type=str)
        if robot_model == 'burger':
            self.wheel_seperation = 0.160 # [m]
            self.turning_radius   = 0.080 # [m]
            self.robot_radius     = 0.105 # [m]
        elif robot_model == 'waffle' or robot_model == 'waffle_pi':
            self.wheel_seperation = 0.287 # [m]
            self.turning_radius   = 0.1435 # [m]
            self.robot_radius     = 0.220 # [m]
        else:
            raise Exception(f'Turtlebot3 model {robot_model} not defined')
        self.wheel_radius   = 0.033 # [m]
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'base_footprint', type=str)

        # Map
        self.occ_threshold = rospy.get_param('~occ_threshold', 50, type=int)
        self.map_frame_id  = None

        # PRM
        show_prm          = rospy.get_param('~show_prm', True, type=bool)
        connection_radius = rospy.get_param('~connection_radius', 1.0, type=float)
        step_size         = rospy.get_param('~step_size', 0.01, type=float)
        num_points        = rospy.get_param('~num_points', 1000, type=int)
        self.prm = PRM(num_points, connection_radius, step_size, show_prm)

        # Publishers and subscribers
        self.path_pub    = rospy.Publisher('path', Path, latch=True, queue_size=10)
        self.costmap_pub = rospy.Publisher('costmap', OccupancyGrid, latch=True, queue_size=10)

        self.goal_sub    = rospy.Subscriber('goal', PoseStamped, self.goalCallback, queue_size=1)

        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.sleep(0.1)

        # Get map
        use_map_topic = rospy.get_param('~use_map_topic', False, type=bool)
        self.lock = Lock()
        if use_map_topic:
            rospy.map_sub = rospy.Subscriber('map', OccupancyGrid, self.mapCallback, queue_size=1)
        else:
            rospy.wait_for_service('static_map')
            map_client = rospy.ServiceProxy('static_map', GetMap)
            rospy.sleep(0.1)
            res = map_client()
            self.preparePRM(res.map)


    def preparePRM(self, map: OccupancyGrid) -> None:
        '''
        Convert the input map to a costmap to use for planning
        Key steps:
            1) Binarize the map
                a) Keep all unknown values (-1) in place
                b) Make all values < self.occ_threshold to 0
                c) Make all values >= self.occ_threshold to 100
            2) Inflate obstacles by the robot size
            3) Use the inflated map to create a PRM
        '''
        # Save frame ID
        self.map_frame_id = map.header.frame_id

        # Convert map to numpy array
        img = np.array(map.data, dtype=np.int8)
        ind_unknown = img == -1 # save indices of unknown cells

        # Shift map values up by 1 to ensure non-negative
        img += 1 # put unknown at 0
        thresh = self.occ_threshold + 1
        max_val = np.uint8(100) + 1
        min_val = np.uint8(0) + 1
        
        # Binarize occupancy grid
        img = img.astype(np.uint8) # convert to uint8 to use dilate
        img[np.logical_and(img >= thresh, img <= max_val)] = max_val
        img[np.logical_and(img >= min_val, img < thresh)]  = min_val

        # Convert to 2D image
        img = img.reshape((map.info.height, map.info.width))
        
        # Get robot kernel (i.e., shape)
        r = np.int8(np.ceil(self.robot_radius / map.info.resolution)) # robot radius in map cells
        robot_img = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*r+1, 2*r+1), (r,r)) # image of robot shape
        
        # Inflate obstacles using dilate function
        img = cv.dilate(img, robot_img)
        
        # Update map data
        map.data = img.flatten().astype(np.int8) - 1 # shift values back down
        map.data[np.logical_and(ind_unknown, map.data == 0)] = -1 # ensure all unknown cells are back to -1
        
        # Publish costmap
        self.costmap_pub.publish(map)

        # Make PRM
        self.prm.buildRoadmap(map)


    def mapCallback(self, msg: OccupancyGrid) -> None:
        '''
        Use the incoming map to create a PRM
        '''
        with self.lock:
            self.preparePRM(msg)
    

    def lookupTransform(self, base_frame: str, child_frame: str, stamp: rospy.Time, time_travel: Optional[rospy.Duration]=rospy.Duration(0)) -> Union[None, np.ndarray]:
        '''
        Look up transformation from base frame to child frame at a given time
        '''
        try:
            trans = self.tf_buffer.lookup_transform(base_frame, child_frame, stamp - time_travel, rospy.Duration(2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logwarn("TF lookup failed: ", error)
            return None
        return t2d.transform2homogeneous(trans.transform)


    def goalCallback(self, goal: PoseStamped) -> None:
        '''
        Use the PRM to plan a path from the current pose of the robot to the goal pose
        '''
        # Look up current pose of the robot in the map frame
        # TODO use lookupTransform function
        pass

        # Plan a path
        # TODO use prm.query to plan a path from the current robot position to the goal position
        # NOTE the query needs to be called inside of `with self.lock:` to ensure that the PRM does not get modified while being used
        # with self.lock:
        #     self.prm.query(<INPUTS>)
        pass

        # Publish the path
        # TODO send the path out using the ROS publisher


if __name__ == '__main__':
    # Initialize objects
    rospy.init_node('global_planning')
    gp = GlobalPlanner()
    rospy.spin()
