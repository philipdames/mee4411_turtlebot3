#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np

import map_conversions as mc
import create_occ_grid as cog

def env_to_occ_grid():
    ##### YOUR CODE STARTS HERE #####
    pass
    ##### YOUR CODE ENDS HERE   #####

if __name__ == '__main__':
    try:
        env_to_occ_grid()
    except rospy.ROSInterruptException:
        pass

