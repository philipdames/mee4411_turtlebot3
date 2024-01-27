#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import LaserScan

class LaserRepub:
    def __init__(self) -> None:
        self.frame_id = rospy.get_param("frame_id", "base_scan")
        self.pub = rospy.Publisher('~scan_out', LaserScan, queue_size=10)
        rospy.sleep(0.1)
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback)

    def callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

def scan_repub():
    rospy.init_node('scan_repub')
    lr = LaserRepub()
    rospy.spin()

if __name__ == '__main__':
    try:
        scan_repub()
    except rospy.ROSInterruptException:
        pass

