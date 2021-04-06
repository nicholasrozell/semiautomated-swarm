#!/usr/bin/env python3
import rospy
import numpy as np
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import HomePosition
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

from FrameConversions import Frame
from graphV2 import Graph
from algorithmsV3 import MiniRRT as RRT
from utils import dist

from shapely.geometry.point import Point
from shapely.geometry import Polygon


class PathPlanning:
    def __init__(self):
        self.home_pos_data = np.zeros(shape=(3,))
        self.position = np.zeros(shape=(3,1))
        self.frame = Frame()
        self.heading = None
        self.home_set = False

        print('\nINITIALIZING')

    def home_pos_cb(self, data):
        # Update only if new home is set in controller
        # Changed rounding factor to 2 thru numpy - Nick
        # Needs to be increased - Nick
        if (np.round(data.geo.latitude, 2) != np.round(self.home_pos_data[0], 2) or
                np.round(data.geo.longitude, 2) != np.round(self.home_pos_data[1], 2) or
                np.round(data.geo.altitude, 2) != np.round(self.home_pos_data[2], 2)):
            print("CALLBACK\n")
            self.home_pos_data[0] = data.geo.latitude
            self.home_pos_data[1] = data.geo.longitude
            self.home_pos_data[2] = data.geo.altitude

            self.frame.addRefLLA(self.home_pos_data)
            print('Reference set\n')
            self.home_set = True

    def global_heading(self, data):
        self.heading = data

    def global_position(self, data):
        self.position[0] = data.latitude
        self.position[1] = data.longitude
        self.position[2] = data.altitude
        self.position = self.frame.ConvLLA2NED(self.position)

        self.pos = np.ndarray.tolist(self.position.reshape((3,)))
        self.pos[2] = -50
        self.pos = tuple(self.pos)

    def main(self):
        """
        dims : graph's dimensions (numpy array of tuples)
        obstacles : obstacles to be placed in the graph (list of shapely.Polygons)
        home : home location (tuple)
        init_state : start location (tuple)
        goal_state : goal location (tuple)
        delta : distance between nodes (int)
        k : shrinking ball facotr (int)
        n : number of waypoints to push (int)
        path : list of waypoints (list of tuples)
        case : case number to set behavior (int)

        Units: meters
        """
        rospy.init_node('RRTnode')
        rospy.wait_for_service('/control/waypoints')
        wp_push = rospy.ServiceProxy('/control/waypoints', WaypointPush)

        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_pos_cb)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.global_heading)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position)

        rate = rospy.Rate(1)    # send msgs at 1 Hz
        start_time = rospy.get_time()
        rate.sleep()


        dims = np.array([(-500, 2500), (-100, 2900), (-50, -50)])
        obstacles = [(Point(1250, 1000).buffer(200)), 
                     (Point(1500, 1500).buffer(200)),
                     (Point(750, 1250).buffer(200)),
                     (Point(800, 650).buffer(200)),
                     (Point(1500, 500).buffer(200)),
                     (Point(1800, 1000).buffer(200)),
                     (Point(2000, 1400).buffer(200)),
                     (Point(1200, 1800).buffer(200))]  
        # obstacles = [Point(2000, 1400).buffer(1200)]      
        init = self.pos
        goal = (2300.0, 2600.0, -50.0)
        delta = 100
        k = 2

        graph = Graph(dims, obstacles)
        path = None
        obstacles = None
        previous_path = None
        trail = []

        start_index = 0

        while not self.home_set:
            rate.sleep()
        
        print('Calculating Trajectory...\n')
        count = 0
        while not rospy.is_shutdown():
            print(count)
            if graph.num_nodes() == 0:
                rrt = RRT(graph, init, goal, delta, k, path)
            if graph.num_nodes() <= 250:
                path = rrt.search()
 
            else:
                init = path[path.index(rrt.brute_force(self.pos, path))+1]
                trail.append(path)
                # print(trail, '\n')

                rate.sleep()
                graph.clear()

                pathNED = np.asarray(path)

                wp_msg = []

                pathLLA = self.frame.ConvNED2LLA(pathNED.T)
                for i in range(len(pathNED)):
                    wp_point = Waypoint()
                    wp_point.frame = 3

                    wp_point.x_lat = pathLLA[0][i]
                    wp_point.y_long = pathLLA[1][i]
                    wp_point.z_alt = pathLLA[2][i]
                    wp_msg += [wp_point]

                # print(wp_msg)
                resp = wp_push(start_index, wp_msg)
                print(resp,'\n')

                # if previous_path != path:
                start_index += path.index(rrt.brute_force(self.pos, path))+1
                    # previous_path = path

                rate.sleep()

            if path is not None:
                if goal in path or dist(self.pos, goal) <= delta*3:
                    print('Goal Reached')
                    print(trail)
                    break
            count += 1
        
            # self.graph.update_obstacles()

if __name__ == '__main__':
    pp = PathPlanning()
    pp.main()
