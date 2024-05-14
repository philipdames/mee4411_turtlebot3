#!/usr/bin/env python3

# import modules
import numpy as np
import torch
import threading
from typing import Tuple

# ros:
import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

# DNN model
from dnn_model import load_dnn_params, DnnNet, set_seed
from dnn_model import normalize_scan, normalize_sub_goal, normalize_final_goal, unnormalize_velocities

from pure_pursuit import PurePursuit
from transform2d_utils import lookup_transform

# do not modify
seed = 1337
set_seed(seed)

#------------------------------------------------------------------------------
#
# the main program starts here
#
#------------------------------------------------------------------------------
class DNNNavigationNode(PurePursuit):
    # Constructor
    def __init__(self):
        # ROS Parameters
        lookahead   = rospy.get_param('~lookahead', 5.0) # lookahead distance [m]
        goal_margin = rospy.get_param('~goal_margin', 3.0) # maximum distance to goal before stopping [m]
        robot_model = rospy.get_param('tb3_model', '')
        robot_frame_id = rospy.get_param('~robot_frame_id', 'base_footprint')

        # Initialize PurePursuit object
        super(DNNNavigationNode, self).__init__(lookahead, goal_margin, robot_model, robot_frame_id)

        # Initialize data:
        self.params = load_dnn_params(rospy.get_param('~param_file'))

        # Data from subscribers
        self.scan_lidar = None
        self.has_scan = False

        self.path = None # store the path to the goal
        self.has_path = False

        self.lock = threading.Lock() # lock to keep data thread safe

        # Set the pytorch device to use
        device = rospy.get_param('~device', None)
        if device is None:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        elif device in ['cpu', 'gpu']:
            self.device = torch.device(device)
        else:
            rospy.logerr('Device {device} not valid')
            return

        # Load model:
        assert rospy.has_param('~model_file')
        model_file = rospy.get_param('~model_file')
        self.model = DnnNet(in_channels=1, 
                            num_hiddens=self.params['num_channels'])
        # moves the model to the device
        self.model.to(self.device)
        # load the weights
        checkpoint = torch.load(model_file, map_location=self.device)
        self.model.load_state_dict(checkpoint['model'])
        # set the model to evaluate
        self.model.eval()
        rospy.loginfo('Finish loading model')

        # Timer:
        self.timer = None
        self.rate = rospy.get_param('~rate', 5.0)

        # Initialize ROS objects
        self.path_sub     = rospy.Subscriber('path', Path, self.path_callback)
        self.scan_sub     = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.tfBuffer     = tf2_ros.Buffer()
        self.cmd_vel_pub  = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=False)
        self.goal_vis_pub = rospy.Publisher('controller_marker', MarkerArray, queue_size=1, latch=True)


    def ready(self) -> bool:
        '''
        Ready to start going once you have a scan and a path
        '''
        with self.lock:
            is_ready = self.has_path and self.has_scan
        return is_ready


    def scan_callback(self, msg: LaserScan) -> None:
        '''
        Callback function for the lidar data
        '''
        with self.lock:
            # get the laser scan data:
            self.scan_data = np.array(msg.ranges, dtype=np.float32)
            self.scan_data[np.isnan(self.scan_data)] = msg.range_max
            self.scan_data[np.isinf(self.scan_data)] = msg.range_max
            self.has_scan = True
        # start the timer if this is the first path received
        if self.timer is None and self.ready():
            self.start()


    def path_callback(self, msg: Path) -> None:
        '''
        Callback function for the path data
        '''
        # lock this data to ensure that it is not changed while other processes are using it
        with self.lock:
            self.path = msg
            self.has_path = True
        # start the timer if this is the first path received
        if self.timer is None and self.ready():
            self.start()


    def start(self) -> None:
        '''
        Start the timer that calculates command velocities
        '''
        # initialize timer for controller update
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)


    def publish_pure_pursuit_markers(self, goal) -> None:
        '''
        Publish markers showing the controller goal
        '''
        # Update markers
        self.update_goal_marker(self.path.header.frame_id, goal, rospy.Time.now())

        # Create and publish marker array
        ma = MarkerArray()
        ma.markers.append(self.goal_marker)
        ma.markers.append(self.goal_marker)
        self.goal_vis_pub.publish(ma)


    def timer_callback(self, event) -> None:
        '''
        Function that runs every time the timer finishes to ensure that controller data are sent regularly
        '''
        with self.lock:
            scan = np.copy(self.scan_data)
            try:
                (x, y, theta) = lookup_transform(self.tfBuffer, self.path.header.frame_id, self.robot_frame_id, rospy.Time.now(), format='xyt')
            except Exception as error:
                rospy.logerr(f'Cannot find goal: {error}')
                return
            if self.params['goal_input'] == 'sub_goal':
                sub_goal = self.findGoal(self.path, x)
            else:
                sub_goal = None
            final_goal = np.array([self.path.poses[-1].pose.position.x, self.path.poses[-1].pose.position.y])
        
        # Publish goal marker
        self.publish_pure_pursuit_markers(sub_goal if self.params['goal_input'] == 'sub_goal' else final_goal)

        # Put into local coordinate frame
        if self.params['goal_input'] == 'sub_goal':
            sub_goal   = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]).dot(sub_goal - x)
        else:
            final_goal = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]]).dot(final_goal - x)   

        # Set velocity command
        cmd_vel = Twist()
        reached_goal = False
        if np.linalg.norm(final_goal) <= self.goal_margin:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            reached_goal = True
        else:
            # dnn inference:
            try:
                vx, wz = self.dnn_inference(scan, sub_goal if self.params['goal_input'] == 'sub_goal' else final_goal)
            except:
                return
            # calculate the goal velocity of the robot and send the command
            cmd_vel.linear.x  = vx
            cmd_vel.angular.z = wz

        # Ensure data is valid before sending
        if np.isnan([cmd_vel.linear.x, cmd_vel.angular.z]).any() or np.isinf([cmd_vel.linear.x, cmd_vel.angular.z]).any():
            rospy.logerr('Velocities are nan or inf')
        else:
            self.cmd_vel_pub.publish(cmd_vel)

        # Stop if reached goal
        if reached_goal:
            rospy.loginfo('Reached goal')
            self.path = None
            self.has_path = False
            self.timer.shutdown()
            self.timer = None


    def dnn_inference(self, scan: np.array, goal: np.array) -> Tuple[float, float]:
        '''
        Use the DNN to calculate the velocity commands
        '''
        # normalize input data:
        scan = normalize_scan(scan, self.params['normalization_method'])
        if self.params['goal_input'] == 'sub_goal':
            goal = normalize_sub_goal(goal, self.params['normalization_method'])
        else:
            goal = normalize_final_goal(goal, self.params['normalization_method'])

        # switch to torch tensor
        batch_scan = torch.tensor(scan, dtype=torch.float32).to(self.device)  
        batch_goal = torch.tensor(goal, dtype=torch.float32).to(self.device)
        
        # run DNN model
        with torch.no_grad():
            vel_cmd = self.model(batch_scan, batch_goal)

        # unnormalize
        vx, wz = unnormalize_velocities(vel_cmd.data.cpu().numpy().flatten())
        
        return vx, wz
