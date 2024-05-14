import rospy
from nav_msgs.msg import OccupancyGrid

from .map_conversions import MapConversions
from .occupancy_grid_map import OccupancyGridMap


def OccupancyGridNode():
    ##### YOUR CODE STARTS HERE #####
    # TODO Set up the ROS node
    pass

    # TODO Set up the ROS publisher for the occupancy grid map
    # NOTE make sure to set latch=True so that the publisher will send the message to any other node that needs it
    pass

    # TODO Read in the map information from the ROS parameter server
    pass

    # TODO Create an OccupancyGridMap based on the provided data using occupancy_grid_utils
    pass

    # TODO Create and publish a nav_msgs/OccupancyGrid msg
    pass
    ##### YOUR CODE ENDS HERE   #####
    rospy.spin() # ensures your node will keep running
