#!/usr/bin/env python3

import sys
import rospy
from pymavlink import mavutil

from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, Quaternion
from mavros_msgs.msg import VFR_HUD, Waypoint
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, VehicleInfoGet
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler
import time

from math import sqrt, pi

class Link:
    def __init__(self):
        rospy.loginfo_once('Initializing Serial Link')
        rospy.init_node('SerialNode')

        rospy.Subscriber('/mavros/battery', BatteryState, self.BatteryState_Cb)
        rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.VFRHUD_Cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.Orientation_Cb)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.GlobalPos_Cb)
        rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, self.GPSVel_Cb)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.RelativeAlt_Cb)

        rospy.Subscriber('/control/path', Waypoint, self.WPInfo_Cb)

        '''
        rospy.wait_for_service('/mavros/param/get')
        try:
            ParamGet_srv = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            MavId_Resp = ParamGet_srv("SYSID_THISMAV")
            SRC_SYSTEM = MavId_Resp.value.integer
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            sys.exit(-1)
        '''
        SRC_SYSTEM = rospy.get_param("/mavros/target_system_id")

        DEVICE = "/dev/ttyUSB0"
        BAUDRATE = 115200

        # Setup serial link
        self.master = mavutil.mavlink_connection(DEVICE, baud=BAUDRATE)
        self.master.mav.srcSystem = SRC_SYSTEM
        self.master.mav.srcComponent = 1

        # Intialize multiple variables
        self.battery_perc, self.battery_volt, self.battery_curr = [0]*3
        self.airspeed, self.groundspeed, self.heading, self.throttle, self.altitude, self.climb = [None]*6
        self.roll, self.pitch, self.yaw, self.rollspeed, self.pitchspeed, self.yawspeed = [None]*6
        self.lat, self.lon, self.alt, self.vx, self.vy, self.vz, self.rel_alt = [None]*7
        self.currX, self.currY, self.currZ, self.WPX, self.WPY, self.WPZ, self.wp_no, self.wp_counter = [0,0,0,-433,1231,-998,1,-10]

        self.Run()

    def VFRHUD_Cb(self, data):
        self.airspeed = data.airspeed
        self.groundspeed = data.groundspeed
        self.heading = data.heading
        self.throttle = data.throttle
        self.climb = data.climb
        self.altitude = data.altitude


    def BatteryState_Cb(self, data):
        self.battery_perc = data.percentage
        self.battery_curr = data.current
        self.battery_volt = data.voltage

    def Orientation_Cb(self, data):
        # I did transformations using trail and error. Need to derive the correct formulae for this
        # coordinate transform of the data.orientation quaternion from ENU -> NED frame
        Q1 = quaternion_from_euler(pi, 0, pi/2.0)
        Q2 = [data.orientation.x, data.orientation.y,
                        data.orientation.z, data.orientation.w]
        Q3 = quaternion_multiply(Q1,Q2)

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(Q3)

        self.roll += 3.14

        self.rollspeed = data.angular_velocity.x
        self.pitchspeed = data.angular_velocity.y
        self.yawspeed = data.angular_velocity.z

    def GlobalPos_Cb(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.alt = data.altitude

    def GPSVel_Cb(self, data):
        self.vx = data.twist.linear.x
        self.vy = data.twist.linear.y
        self.vz = data.twist.linear.z

    def RelativeAlt_Cb(self, data):
        self.rel_alt = data.data

    def WPInfo_Cb(self, data):
        self.currX = data.x_lat
        self.currY = data.y_long
        self.currZ = data.z_alt

        self.WPX = data.param2
        self.WPY = data.param3
        self.WPZ = data.param4

        self.wp_no = data.param1
        self.wp_counter = data.command

    def Run(self):
        rospy.wait_for_service('/mavros/vehicle_info_get')
        rate = rospy.Rate(1000)

        LastHeartBeatTime, LastSysStatusTime, LastVFRTime, \
        LastAttitudeTime,LastGlobalPosIntTime, LastNavControlTime = 0, 0, 0, 0, 0, 0

        rate.sleep()
        time.sleep(1)

        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            curr_time = now.secs + now.nsecs/1e9

            if curr_time - LastHeartBeatTime > 1:
                try:
                    VehicleInfo_srv = rospy.ServiceProxy('/mavros/vehicle_info_get', VehicleInfoGet)
                    HeartbeatInfo_srv = VehicleInfo_srv(0, 0, 1)
                    #rospy.loginfo("Sending Heartbeat")
                    apmtype = HeartbeatInfo_srv.vehicles[0].type
                    autopilot = HeartbeatInfo_srv.vehicles[0].autopilot
                    base_mode = HeartbeatInfo_srv.vehicles[0].base_mode
                    custom_mode = HeartbeatInfo_srv.vehicles[0].custom_mode
                    system_status = HeartbeatInfo_srv.vehicles[0].system_status

                    # send a heartbeat
                    self.master.mav.heartbeat_send(apmtype, autopilot, base_mode, custom_mode, system_status)
                    LastHeartBeatTime = curr_time
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)
                    sys.exit(-1)



            if curr_time - LastSysStatusTime > 0.5:
                #rospy.loginfo("Sending System Status")
                self.master.mav.sys_status_send(onboard_control_sensors_present=3,
                                                onboard_control_sensors_enabled=3,
                                                onboard_control_sensors_health=3,
                                                load=0, voltage_battery=int(self.battery_volt*1000),
                                                current_battery=int(self.battery_curr*100),
                                                battery_remaining=int(self.battery_perc*100),
                                                drop_rate_comm=0, errors_comm=0, errors_count1=0, errors_count2=0,
                                                errors_count3=0, errors_count4=0, force_mavlink1=False)

                LastSysStatusTime = curr_time

            if curr_time - LastVFRTime > 0.1:
                #rospy.loginfo("Sending VFR HUD Info")
                self.master.mav.vfr_hud_send(airspeed=self.airspeed,
                                             groundspeed=self.groundspeed,
                                             heading=int(self.heading),
                                             throttle=int(self.throttle*100),
                                             alt=self.altitude,
                                             climb=self.climb)

                LastVFRTime = curr_time

            if curr_time - LastAttitudeTime > 0.05:
                #rospy.loginfo("Sending Attitude message")
                self.master.mav.attitude_send(0,
                                              roll=self.roll,
                                              pitch=self.pitch,
                                              yaw=self.yaw,
                                              rollspeed=self.rollspeed,
                                              pitchspeed=self.pitchspeed,
                                              yawspeed=self.yawspeed)
                LastAttitudeTime = curr_time

            if curr_time - LastGlobalPosIntTime > 0.2:
                #rospy.loginfo("Sending GPS message")
                self.master.mav.global_position_int_send(0,
                                                         lat=int(self.lat*1e7),
                                                         lon=int(self.lon*1e7),
                                                         alt=int(self.alt*1e3),
                                                         relative_alt=int(self.rel_alt*1e3),
                                                         vx=int(self.vx*1e2),
                                                         vy=int(self.vy*1e2),
                                                         vz=int(self.vz*1e2),
                                                         hdg=int(self.heading*1e2))
                LastGlobalPosIntTime = curr_time

            if curr_time - LastNavControlTime > 1:
                '''
                This message is used to send current waypoint and current position information to the GCS
                
                Since mission planner doesnt have customizing capabilities we use redefine variables in this messsage
                to act as Current WP and Current Position
                nav_roll -> CurrentWP X location(North)
                nav_pitch -> CurrentWP Y location(East)
                nav_bearing -> CurrentWp Z location(Down)
                wp_dist -> Current Wp sequence
                target_bearing -> Total Waypoints in sequence
                alt_error -> Current position X
                aspd_error -> Current position Y
                xtrack_error -> Current position Z
                '''
                # rospy.loginfo("Sending Navigation Info")
                self.master.mav.nav_controller_output_send(nav_roll=self.WPX,
                                                           nav_pitch=self.WPY,
                                                           nav_bearing=int(-1*self.WPZ),
                                                           target_bearing=self.wp_counter,
                                                           wp_dist=int(self.wp_no),
                                                           alt_error=self.currX,
                                                           aspd_error=self.currY*100,
                                                           xtrack_error=self.currZ)

                LastNavControlTime = curr_time

            # Check for incoming data on the serial port and count
            # how many messages of each type have been received
            while self.master.port.inWaiting() > 0:
                # recv_msg will try parsing the serial port buffer
                # and return a new message if available
                m = self.master.recv_msg()
                #print(m)

                if m is None:
                    break  # No new message

            # Spin the ros loop
            rate.sleep()


if __name__ == "__main__":
    serialLink = Link()
