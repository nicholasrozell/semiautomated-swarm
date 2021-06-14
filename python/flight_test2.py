import rospy
import numpy as np
import time

from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import HomePosition
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

from FrameConversions import Frame

from shapely.geometry.point import Point

from graph import Graph
from rrt import MiniRRT as RRT
from utils import dist


class PathPlanning:
    def __init__(self):
        print('\nINITIALIZING\n')

        rospy.init_node('RRTnode')
        rospy.wait_for_service('/control/waypoints')

        self.home_pos_data = np.zeros(shape=(3,))
        self.position = np.zeros(shape=(3, 1))
        self.frame = Frame()
        self.heading = None
        self.home_set = False
        self.gps_status = None

        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_pos_cb)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.global_heading)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position)

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
        self.gps_status = data.status.status

        if self.gps_status >= 0:
            self.position[0] = data.latitude
            self.position[1] = data.longitude
            self.position[2] = data.altitude
            self.position = self.frame.ConvLLA2NED(self.position)

            self.pos = np.ndarray.tolist(self.position.reshape((3,)))
            self.pos[2] = -2
            self.pos = tuple(self.pos)
            #print(self.pos)


        #print('GPS Status:  ',self.gps_status)

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

        #rospy.init_node('RRTnode')
        #rospy.wait_for_service('/control/waypoints')

        wp_push = rospy.ServiceProxy('/control/waypoints', WaypointPush)

        #rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_pos_cb)
        #rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.global_heading)
        #rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position)

        rate = rospy.Rate(1)    # send msgs at 1 Hz
        start_time = rospy.get_time()
        rate.sleep()
        #		   E-x	     N-y       D-z
        dims = np.array([(-80, 60), (-30, 40), (-2, -2)])
        #dims = np.array([(-30, 40), (-80, 60), (-2, -2)])
        init = self.pos
        goal = (-60, 20, -2.0)
        delta = 5
        k = 2

        graph = Graph(dims)
        path = None
        trail = []
        position = []

        start_index = 0

        while not self.home_set:
            rate.sleep()

        # while loop for gps_check:
        while self.gps_status == -1 or self.gps_status is None:
            print("BAD GPS")
            rate.sleep()

        print('Calculating Trajectory...\n')
        while not rospy.is_shutdown():
            if graph.num_nodes() == 0:
                #t0 = time.time()
                rrt = RRT(graph, init, goal, delta, k, path, self.heading)
            if graph.num_nodes() <= 250:
                path, leaves = rrt.search()
            else:
                init = path[path.index(rrt.brute_force(self.pos, path))+1]
                trail.append(path)
                position.append(self.pos)

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

                print(wp_msg, '\n')
                resp = wp_push(start_index, wp_msg)
                print(resp, '\n')

                start_index += path.index(rrt.brute_force(self.pos, path))+1
                #print('Search Time: {} secs\n'.format(time.time() - t0))
                rate.sleep()

            if path is not None:
                if dist(self.pos, goal) <= delta*3 or goal in path:
                    print('Goal Reached.\n')
                    print('trail :  ', trail, '\n')
                    print('position:  ', position, '\n')
                    break

            print('EOL')

if __name__ == '__main__':
    pp = PathPlanning()
    pp.main()
