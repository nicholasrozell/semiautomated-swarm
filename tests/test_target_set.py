import rospy
from mavros_msgs.msg import *
from std_msgs.msg import Float64

from pymavlink import mavutil
import math

'''
This code test setting mavros target attitude message
'''


class RadioComm:
    def __init__(self):
        self.autopilot = mavutil.mavlink_connection('udpout:localhost:16552')
        self.autopilot.mav.srcSystem = 1
        self.autopilot.mav.srcComponent = 1

        self.heading_data = 0

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

    def heading_cb(self, data):
        self.heading_data = data.data

    def run(self):
        pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        rospy.Publisher('/mavros/global_position/compass_hdg', Float64, queue_size=10)
        rospy.init_node('commNode', anonymous=True)
        rate = rospy.Rate(100)  # 100hz

        while not rospy.is_shutdown():
            att_msg = AttitudeTarget()

            yaw = self.heading_data

            quat = self.to_quaternion(-60.0, 0.0,
                                      90)  # Set yaw to 90 then servo4 outputs 1500....need to figure out why

            att_msg.type_mask = 0b00000000
            att_msg.orientation.w = quat[0]
            att_msg.orientation.x = quat[1]
            att_msg.orientation.y = quat[2]
            att_msg.orientation.z = quat[3]

            att_msg.thrust = 0.8

            att_msg.body_rate.x = 0
            att_msg.body_rate.y = 0
            att_msg.body_rate.z = 0

            pub.publish(att_msg)
            print("Sending")


            rate.sleep()


if __name__ == '__main__':
    rc = RadioComm()
    rc.run()
