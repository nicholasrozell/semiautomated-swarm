#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush
import pandas as pd
from . import graph
from .algorithms import RRTstar
from .FrameConversions import NED2LLA
import numpy as np

def main():
    rospy.init_node('RRTnode')
    rospy.wait_for_service('/control/waypoints')
    wp_push = rospy.ServiceProxy('/control/waypoints', WaypointPush)

    rate = rospy.Rate(1)    # send msgs at  Hz

    print('INITIALIZING\n')
    start_time = rospy.get_time()

    dims = Graph.np_array((-10, 10), (-10, 10), (-100, -100))
    obstacles = []

    graph = Graph(dims, obstacles)
    home = (0, 0, -100)
    init_state = (0, 0, -100)
    goal_state = (5, 5, -100)
    delta = 1
    k = 11
    n = 5
    path = []
    case = 0
    count = 0


    while not rospy.is_shutdown():
        rrt = RRTstar(graph, init_state, goal_state, delta, k, path, n, case)
        path, case = rrt.search()
        if case == 1:
            init_state = path[n]
        elif case == 2:
            init_state = path[n + n]
        elif case == 3:
            init_state = path[-n]
            goal_state = home

        graph.clear()


        pathNED = np.asarray(path)

        wp_msg = []
        start_index = count

        for i in range(len(pathNED)):
            pathLLA = NED2LLA(pathNED[i])

            wp_point = Waypoint()
            # wp_point.frame = int(df.iloc[i].frame)
            wp_point.frame = 3
            # wp_point.x_lat = df.iloc[i].lat
            wp_point.x_lat = pathLLA[i][0]
            # wp_point.y_long = df.iloc[i]/.long
            wp_point.y_long = pathLLA[i][1]
            # wp_point.z_alt = df.iloc[i].alt
            wp_point.z_alt = pathLLA[i][2]
            wp_msg += [wp_point]

        resp = wp_push(start_index, wp_msg)
        print(resp)
        count += n
        rate.sleep()

if __name__ == '__main__':
    main()