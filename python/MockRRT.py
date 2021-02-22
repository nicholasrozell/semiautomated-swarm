#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush
import pandas as pd

def main():
    rospy.init_node('RRTnode')
    rospy.wait_for_service('/control/waypoints')
    wp_push = rospy.ServiceProxy('/control/waypoints', WaypointPush)

    rate = rospy.Rate(1)  # send msgs at 0.5 Hz
    df = pd.read_csv('SWOMission.csv')


    print("Starting RRT")
    start_time = rospy.get_time()

    #while not rospy.is_shutdown():
    #if rospy.get_time() < 5*60:
    # First 5 mins
    wp_msg = []
    start_index = 0
    for i in range(len(df)):
        wp_point = Waypoint()
        wp_point.frame = int(df.iloc[i].frame)
        wp_point.param1 = 0
        wp_point.param2 = 0
        wp_point.param3 = 0
        wp_point.param4 = 0
        wp_point.x_lat = df.iloc[i].lat
        wp_point.y_long = df.iloc[i].long
        wp_point.z_alt = df.iloc[i].alt
        wp_msg += [wp_point]
    #else:
    #    # After 5 Mins
    #    print("Sending new waypoints")
    #    wp_msg = []
    #    start_index = len(df)
    #    for i in range(len(df1)):
    #        wp_point = Waypoint()
    #        wp_point.frame = int(df.iloc[i].frame)
    #        wp_point.param1 = 0
    #        wp_point.param2 = 0
    #        wp_point.param3 = 0
    #        wp_point.param4 = 0
    #        wp_point.x_lat = df1.iloc[i].lat
    #        wp_point.y_long = df1.iloc[i].long
    #        wp_point.z_alt = df1.iloc[i].alt
    #        wp_msg += [wp_point]

        # add the waypoints in this iteration to wp_msg message and publish
    resp = wp_push(start_index, wp_msg)
    print(resp)
    rate.sleep()

if __name__ == '__main__':
    main()