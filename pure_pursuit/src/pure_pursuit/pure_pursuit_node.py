#!/usr/bin/env python3

import rospy
from pure_pursuit import PurePursuit


if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    
    pp = PurePursuit()
    
    rospy.spin()

