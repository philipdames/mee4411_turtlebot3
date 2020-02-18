#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import LaserScan

pub = []
frame_id = ""

def callback(msg):
    msg.header.frame_id = frame_id
    pub.publish(msg)

def scan_repub():
    rospy.init_node('scan_repub')
    
    global frame_id
    frame_id = rospy.get_param("frame_id", "base_scan")
    
    global pub
    pub = rospy.Publisher('~scan_out', LaserScan, queue_size=10)
    
    rospy.sleep(0.1)
    rospy.Subscriber('scan', LaserScan, callback)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        scan_repub()
    except rospy.ROSInterruptException:
        pass

