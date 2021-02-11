import rospy
import message_filters
from mavros_msgs.msg import *
from mavros_msgs.srv import SetMode
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from geometry_msgs.msg import TwistStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import socket
from pymavlink import mavutil
import math

'''
All the messages that are output

0: HEARTBEAT
1: SYS_STATUS
27: RAW_IMU
29: SCALED_PRESSURE
30: ATTITUDE
33: GLOBAL_POSITION_INT
36: SERVO_OUTPUT_RAW
42: MISSION_CURRENT
62: NAV_CONTROLLER_OUTPUT
65: RC_CHANNELS
74: VFR_HUD
116: SCALED_IMU2
125: POWER_STATUS
129: SCALED_IMU3

'''


class RadioComm:
    def __init__(self):
        self.autopilot = mavutil.mavlink_connection('udpout:localhost:14552')
        self.autopilot.mav.srcSystem = 1
        self.autopilot.mav.srcComponent = 1

        self.GOT_GLOBAL_INT = False
        self.GOT_GLOBAL_HDG = False
        self.GOT_GLOBAL_RELALT = False

        self.global_int_data = None
        self.global_int_relalt = None
        self.global_int_hdg = None

        HOST = '127.0.0.1'
        mavproxy_port = 14552
        self.address = (HOST, mavproxy_port)
        self.mavproxy_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
            """
            Convert degrees to quaternions
            """
            t0 = math.cos(math.radians(yaw * 0.5))
            t1 = math.sin(math.radians(yaw * 0.5))
            t2 = math.cos(math.radians(roll * 0.5))
            t3 = math.sin(math.radians(roll * 0.5))
            t4 = math.cos(math.radians(pitch * 0.5))
            t5 = math.sin(math.radians(pitch * 0.5))

            w = t0 * t2 * t4 + t1 * t3 * t5
            x = t0 * t3 * t4 - t1 * t2 * t5
            y = t0 * t2 * t5 + t1 * t3 * t4
            z = t1 * t2 * t4 - t0 * t3 * t5

            return [w, x, y, z]

    def sys_status_cb(self, data):

        if data.mode == 'MANUAL':
            base_mode = 209
            custom_mode = 0
        elif data.mode == 'GUIDED':
            base_mode = 217
            custom_mode = 15
        elif data.mode == 'STABILIZE':
            base_mode = 209
            custom_mode = 2
        elif data.mode == 'CMODE(13)':
            base_mode = 217
            custom_mode = 13
        else:
            base_mode = 209
            custom_mode = 0

        self.autopilot.mav.heartbeat_send(type=mavutil.mavlink.MAV_TYPE_FIXED_WING,
                                          autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                          base_mode=base_mode, custom_mode=custom_mode, system_status=data.system_status)

    def attitude_cb(self, data):
        time_ms = data.header.stamp.secs*1e3 + data.header.stamp.nsecs*1e-6
        rpy = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.autopilot.mav.attitude_send(0, rpy[0], -1*rpy[1], (2*3.14159265)-(rpy[2]-(3.14159265/2.0)),
                                         data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)

    def global_position_int_cb(self, data):
        if not self.GOT_GLOBAL_INT:
            self.global_int_data = data
            self.GOT_GLOBAL_INT = True

    def global_position_relalt(self, data):
        if not self.GOT_GLOBAL_RELALT:
            self.global_int_relalt = data
            self.GOT_GLOBAL_RELALT = True

    def global_position_hdg(self, data):
        if not self.GOT_GLOBAL_HDG:
            self.global_int_hdg = data
            self.GOT_GLOBAL_HDG = True

    def battery_cb(self, data):
        self.autopilot.mav.sys_status_send(0,0,0,0,int(data.voltage*1000), int(-1*data.current*100), int(data.percentage*100),
                                           0,0,0,0,0,0)

    def hud_cb(self, data):
        time_ms = data.header.stamp.secs*1e3 + data.header.stamp.nsecs*1e-6

        self.autopilot.mav.vfr_hud_send(data.airspeed, data.groundspeed, data.heading, int(data.throttle*100),
                                        data.altitude, data.climb)

    def run(self):
        pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        rospy.Subscriber("/mavros/state", State, self.sys_status_cb)
        rospy.Subscriber("/mavros/imu/data", Imu, self.attitude_cb)

        global_pos_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.global_position_int_cb)
        global_relalt_sub = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.global_position_relalt)
        global_hdg_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.global_position_hdg)

        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.hud_cb)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_cb)

        rospy.init_node('commNode', anonymous=True)
        rate = rospy.Rate(100)  # 100hz

        rospy.wait_for_service('/mavros/set_mode')
        #try:
        #    setFCmode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        #    resp1 = setFCmode('217', '15')
        #    print("Set mode response is ", resp1)
        #except rospy.ServiceException as e:
        #    print("Service call failed: %s" % e)

        while not rospy.is_shutdown():

            if self.GOT_GLOBAL_INT and self.GOT_GLOBAL_HDG and self.GOT_GLOBAL_RELALT:

                lat = int(self.global_int_data.latitude * 1e7)
                lon = int(self.global_int_data.longitude * 1e7)
                alt = int(self.global_int_data.altitude * 1e3)
                relalt = int(self.global_int_relalt.data*1e3)
                hdg = int(self.global_int_hdg.data * 1e2)

                self.autopilot.mav.global_position_int_send(0, lat, lon, alt, relalt, 0, 0, 0, hdg)

                self.GOT_GLOBAL_INT = False
                self.GOT_GLOBAL_HDG = False
                self.GOT_GLOBAL_RELALT = False

            #print("Sending")
            att_msg = AttitudeTarget()

            quat = self.to_quaternion(-50, 20, 180)

            att_msg.type_mask = 0b00000100
            att_msg.orientation.w = quat[0]
            att_msg.orientation.x = quat[1]
            att_msg.orientation.y = quat[2]
            att_msg.orientation.z = quat[3]

            att_msg.thrust = 0.7

            att_msg.body_rate.x = 0
            att_msg.body_rate.y = 0
            att_msg.body_rate.z = 0

            pub.publish(att_msg)
            rate.sleep()


if __name__ == '__main__':
    rc = RadioComm()
    rc.run()