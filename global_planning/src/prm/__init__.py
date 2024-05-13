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

from .prm import PRM
import tb3_utils.transform2d as t2d
from tb3_utils.tb3_params import TB3Params


class PRMNode:
    def __init__(self) -> None:
        # Robot parameters
        robot_model = rospy.get_param('tb3_model', "")
        self.params = TB3Params(robot_model)
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'base_footprint')

        # Map
        self.occ_threshold = rospy.get_param('~occ_threshold', 50)
        self.map_frame_id  = None

        # PRM
        show_prm          = rospy.get_param('~show_prm', True)
        connection_radius = rospy.get_param('~connection_radius', 1.0)
        step_size         = rospy.get_param('~step_size', 0.01)
        num_points        = rospy.get_param('~num_points', 1000)
        self.prm = PRM(num_points, connection_radius, step_size, show_prm)

        # Publishers and subscribers
        self.path_pub    = rospy.Publisher('path', Path, latch=True, queue_size=10)
        self.costmap_pub = rospy.Publisher('costmap', OccupancyGrid, latch=True, queue_size=10)

        self.goal_sub    = rospy.Subscriber('goal', PoseStamped, self.goal_callback, queue_size=1)

        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.sleep(0.1)

        # Get map
        use_map_topic = rospy.get_param('~use_map_topic', False)
        self.lock = Lock()
        if use_map_topic:
            rospy.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback, queue_size=1)
        else:
            rospy.wait_for_service('static_map')
            map_client = rospy.ServiceProxy('static_map', GetMap)
            rospy.sleep(0.1)
            res = map_client()
            self.prepare_prm(res.map)


    def prepare_prm(self, map: OccupancyGrid) -> None:
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


    def map_callback(self, msg: OccupancyGrid) -> None:
        '''
        Use the incoming map to create a PRM
        '''
        with self.lock:
            self.preparePRM(msg)


    def goal_callback(self, goal: PoseStamped) -> None:
        '''
        Use the PRM to plan a path from the current pose of the robot to the goal pose
        '''
        ##### YOUR CODE STARTS HERE #####
        # TODO Look up current pose of the robot in the map frame
        pass

        # TODO Plan a path using prm.query to plan a path from the current robot position to the goal position
        # NOTE the query needs to be called inside of `with self.lock:` to ensure that the PRM does not get modified while being used
        # with self.lock:
        #     self.prm.query(<INPUTS>)
        pass

        # TODO Publish the path using the ROS publisher
        pass
        ##### YOUR CODE ENDS HERE   #####
