#!/usr/bin/env python3

import rospy
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointPushResponse, StreamRate
from nav_msgs.msg import Odometry
import numpy as np
import time

class QuadController:
    def __init__(self):
        rospy.loginfo_once('Initializing Quad GNC')
        rospy.init_node('QuadGNC_node')

        rospy.wait_for_service('/mavros/set_stream_rate')
        try:
            streamrate_srv = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
            str_out = streamrate_srv(0, 100, 1)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            sys.exit(-1)

        wp_srv = rospy.Service('/control/waypoints', WaypointPush, self.WaypointHandle)
        #rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.Position_cb)
        rospy.Subscriber('/mavros/global_position/local', Odometry, self.Position_cb)

        #self.pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        self.pub_wpinfo = rospy.Publisher('/control/path', Waypoint, queue_size=10)

        # init some variables
        self.X, self.Y, self.Z = [0]*3
        # Create a list to hold the waypoints
        self.WpList = {}

    def Position_cb(self, data):
        #self.X = data.latitude
        #self.Y = data.longitude
        #self.Z = data.altitude

        self.X = data.pose.pose.position.y
        self.Y = data.pose.pose.position.x
        self.Z = data.pose.pose.position.z

    def WaypointHandle(self, req):
        # get the start sequence
        start_index = req.start_index
        counter = 0
        for i in range(start_index, start_index + len(req.waypoints)):
            self.WpList[i] = req.waypoints[counter]
            counter += 1

        return WaypointPushResponse(True, len(req.waypoints))

    def Run(self):
        rate = rospy.Rate(10)

        tgt_msg = PositionTarget()
        wpinfo_msg = Waypoint()

        rate.sleep()

        time.sleep(2)

        Counter = 1
        while not rospy.is_shutdown():
            if len(self.WpList) > 0:

                #print(self.WpList[Counter])

                tgt_msg.coordinate_frame = 1 #5
                tgt_msg.type_mask = 4088

                # since data comes in NED form we need to convert to ENU. switch x and y and invert sign of z
                tgt_msg.position.x = self.WpList[Counter].y_long
                tgt_msg.position.y = self.WpList[Counter].x_lat
                tgt_msg.position.z = -1*self.WpList[Counter].z_alt


                wpinfo_msg.x_lat = self.X
                wpinfo_msg.y_long = self.Y
                wpinfo_msg.z_alt = self.Z

                wpinfo_msg.param2 = self.WpList[Counter].x_lat
                wpinfo_msg.param3 = self.WpList[Counter].y_long
                wpinfo_msg.param4 = self.WpList[Counter].z_alt
                #print(tgt_msg.position.z, wpinfo_msg.param4)
                wpinfo_msg.param1 = Counter
                wpinfo_msg.command = len(self.WpList)

                #print(tgt_msg.position.x, tgt_msg.position.y, tgt_msg.position.z)
                self.pub_wpinfo.publish(wpinfo_msg)

                #print("--------------------------------------")
                if np.sqrt((self.X - self.WpList[Counter].x_lat)**2 + (self.Y - self.WpList[Counter].y_long)**2) > 5:
                    if self.Z > -2:
                        self.pub.publish(tgt_msg)
                        #print("Publishing")
                else:
                    if Counter != len(self.WpList) - 1:
                        Counter += 1
                    #else:
                    #    rospy.loginfo_once("Done")
                    #    break

            rate.sleep()


if __name__ == '__main__':
    quadcontrol = QuadController()
    quadcontrol.Run()


