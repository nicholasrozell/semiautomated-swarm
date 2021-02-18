#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import WaypointList, Waypoint
import pandas as pd

def main():
    rospy.init_node('RRTnode')
    pub = rospy.Publisher('/control/waypoints', WaypointList, queue_size=1)

    rate = rospy.Rate(1)  # send msgs at 0.5 Hz
    df = pd.read_csv('Wpdata3.csv')

    print("Starting RRT")

    while not rospy.is_shutdown():
        wp_msg = WaypointList()
        wp_msg.current_seq = 0
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
            wp_msg.waypoints += [wp_point]

        # add the waypoints in this iteration to wp_msg message and publish
        pub.publish(wp_msg)
        rate.sleep()

if __name__ == '__main__':
    main()