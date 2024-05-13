import rospy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray

import networkx as nx
import numpy as np
from scipy.spatial import KDTree

import map_conversions as mc

Z = 0.0 # z coordinate for the graph display
ALPHA = 0.25 # alpha value for graph transparency

class PRM:
    def __init__(self, n_points: float, connection_radius: float, step_size: float, show_prm: bool = True) -> None:
        # Parameters
        self.n_points          = n_points # number of points in the PRM
        self.connection_radius = connection_radius # radius in which to check for connections
        self.step_size         = step_size # size of step to take for collision checking
        self.show_prm          = show_prm # show the PRM in rviz or not

        # Data structures
        self.map = None # costmap
        self.graph = None

        # ROS publisher
        if self.show_prm:
            self.marker_pub = rospy.Publisher('~graph', MarkerArray, latch=True, queue_size=10)


    def buildRoadmap(self, map: OccupancyGrid) -> None:
        '''
        Build the probabilistic roadmap in the given occupancy grid
        '''
        # Save map
        self.map = map
        
        # Build graph
        self.graph = nx.Graph() # intialize empty graph
        
        for i in range(self.n_points):
            # Generate valid node
            # TODO generate a point in free space 
            pass

            # TODO add the point to the graph node list with an attribute called 'location' holding the 2D position
            #       'location' can be formatted as a list or as a numpy array
            #       see documentation here: https://networkx.org/documentation/stable/tutorial.html#adding-attributes-to-graphs-nodes-and-edges
            pass

            # Connect to other nodes in the graph
            # TODO find other nodes within the connection radius
            pass
        
            # TODO check to see if the path from the new point to any node is obstacle free
            pass

            # TODO if it is a clear path, add it to the graph edge list
            pass
            
            # Display graph as it is being built
            if i % 100 == 99:
                self.show()
        
        # Show final graph
        self.show()

        # Initialize KD tree for quickly finding nearest node
        pts = np.array(self.graph.nodes.data('location'))
        assert pts.shape[1] == 2
        self.kdtree = KDTree(pts)


    def sampleFreePoint(self) -> np.array:
        '''
        Draw a random points from within the free space of self.map
        Return a 2D point within the map as a numpy.array
        '''
        # Draw a random point within the boundary
        pt = np.random.rand(2) # TODO make this correct

        # Check if point is valid (i.e., not in collision based on self.map)
        # TODO figure out if pt is in collision, if it is try again, if not then return the point
        pass

        return pt


    def validEdge(self, p0: np.array, p1: np.array) -> bool:
        '''
        Check to see if an edge connection p0 to p1 is in collision with the map
        '''
        # TODO create a series of points starting at p0 and ending at p1 in steps of self.step_size
        pass
        # TODO Check to make sure none of the points collide with the map
        pass

        return False # TODO fix this


    def findNearestNode(self, pt: np.array) -> int:
        '''
        Find the nearest node in the graph (with a valid edge) to the input point pt
        Return the index of the node
        '''
        _, ind = self.kdtree.query(pt, k = 10)
        for i in ind:
            if self.validEdge(pt, self.graph.nodes[i]['location']):
                return i
        return None


    def query(self, start: np.array, goal: np.array) -> Path:
        '''
        Query the PRM to get a path from a start point to a goal point
        Return a nav_msgs/Path object from start to goal
        '''
        # Make sure the PRM is initialized
        if self.graph is None:
            raise Exception('PRM not initialized')
        
        # Find nearest nodes to start and goal
        n_start = self.findNearestNode(start)
        if n_start is None:
            raise Exception('Start point invalid')
        n_goal = self.findNearestNode(goal)
        if n_goal is None:
            raise Exception('Goal point invalid')
        
        # Plan path using A*
        # TODO use networkx library to call A* to find a path from n_start to n_goal
        pass

        # Generate returned path
        # TODO convert the path returned by networkx to a nav_msgs/Path
        # NOTE make sure to include the start and goal points
        pass
        
        return Path() # TODO fix this


    def show(self) -> None:
        rospy.loginfo('Graph has %d nodes', self.graph.number_of_nodes())
        if not self.show_prm:
            return
        
        # Create marker to show the nodes in the graph
        points_marker = Marker()
        points_marker.header.frame_id = self.map.header.frame_id
        points_marker.header.stamp = rospy.Time.now()
        points_marker.ns = 'points'
        points_marker.type = Marker.SPHERE_LIST
        points_marker.pose.orientation.w = 1.0
        points_marker.scale.x = 0.1
        points_marker.scale.y = 0.1
        points_marker.scale.z = 0.1
        points_marker.color.r = 1.0
        points_marker.color.a = ALPHA

        for _, loc in self.graph.nodes.data('location'):
            points_marker.points.append(Point(*loc, Z))

        # Create marker to show the edges in the graph
        edges_marker = Marker()
        edges_marker.header.frame_id = self.map.header.frame_id
        edges_marker.header.stamp = rospy.Time.now()
        edges_marker.ns = 'edges'
        edges_marker.type = Marker.LINE_LIST
        edges_marker.pose.orientation.w = 1.0
        edges_marker.scale.x = 0.05
        edges_marker.color.b = 1.0
        edges_marker.color.a = ALPHA

        for e in self.graph.edges:
            p0 = self.graph.nodes[e[0]]['location']
            p1 = self.graph.nodes[e[1]]['location']
            edges_marker.points.append(Point(*p0, Z))
            edges_marker.points.append(Point(*p1, Z))
        
        # Publish marker
        ma = MarkerArray()
        ma.markers.append(points_marker)
        ma.markers.append(edges_marker)
        self.marker_pub.publish(ma)
