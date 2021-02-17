#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import WaypointList

def main():
    rospy.init_node('RRTnode')
    pub = rospy.Publisher('/control/waypoints',WaypointList, queue_size=1)

    rate = rospy.Rate(1) # send msgs at 1 Hz

    while not rospy.is_shutdown():
        wp_msg = WaypointList()

        # add the waypoints in this iteration to wp_msg message and publish

        pub.Publish(wp_msg)
        rate.sleep()