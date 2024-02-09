#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def block_vis():
    # initialize ROS node and publisher
    rospy.init_node('block_vis')
    blocks_pub = rospy.Publisher('env_markers', MarkerArray, queue_size=1, latch=True)

    # read in parameters
    boundary = rospy.get_param('~boundary')
    blocks = rospy.get_param('~blocks')
    frame_id = rospy.get_param('~frame_id')
    alpha = 0.5
    if rospy.has_param('~alpha'):
        alpha = rospy.get_param('~alpha')
        
    # print map information to screen
    rospy.loginfo("boundary: %.2f, %.2f, %.2f, %.2f", boundary[0], boundary[1], boundary[2], boundary[3])
    for blk in blocks:
        rospy.loginfo("block: %.2f, %.2f, %.2f, %.2f", blk[0], blk[1], blk[2], blk[3])
    
    # initialize header
    h = Header()
    h.frame_id = frame_id
    h.stamp = rospy.Time.now()
    
    # initialize block marker message
    ma = [] # marker array
    
    # initialize boundary wall blocks
    thickness = 0.05 # thickness of walls
    height = 0.5 # height of markers
    for i in range(0,4):
        m = Marker();
        m.header = h
        m.ns = "boundary"
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        # ensure all area inside of the boundary is in the map
        if i % 2 == 0:
            m.pose.position.x = (boundary[0]+boundary[2])/2.
            m.pose.position.y = boundary[i+1] - (-1)**(i/2.) * thickness/2.
            m.scale.x = boundary[2] - boundary[0] + 2. * thickness
            m.scale.y = thickness
        else:
            m.pose.position.x = boundary[i-1] - (-1)**((i-1.)/2.) * thickness/2.
            m.pose.position.y = (boundary[1]+boundary[3])/2.
            m.scale.x = thickness
            m.scale.y = boundary[3] - boundary[1] + 2. * thickness
        m.pose.position.z = height/2.
        m.pose.orientation.w = 1.0
        m.scale.z = height
        m.color.r = 157./255. # Temple cherry color
        m.color.g = 34./255.
        m.color.b = 53./255.
        m.color.a = 1.0

        ma.append(m)
    
    # initialize block markers
    i = 0
    for b in blocks:
        m = Marker();
        m.header = h
        m.ns = "blocks"
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = (b[0]+b[2])/2.
        m.pose.position.y = (b[1]+b[3])/2.
        m.pose.position.z = height/2.
        m.scale.x = b[2] - b[0]
        m.scale.y = b[3] - b[1]
        m.scale.z = height
        m.color.r = 157./255. # Temple cherry color
        m.color.g = 34./255.
        m.color.b = 53./255.
        m.color.a = alpha # make slightly transparent
        
        ma.append(m)
        i = i + 1
    
    blocks_pub.publish(MarkerArray(markers=ma))
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        block_vis()
    except rospy.ROSInterruptException:
        pass

