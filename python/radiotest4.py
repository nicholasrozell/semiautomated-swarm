import rospy
from mavros_msgs.msg import *
from std_msgs.msg import Float64

from pymavlink import mavutil
import math

'''
This code proves that when mavproxy is called with two masters, one source from pixhawk and the second from
the planning code, mavproxy streams all messages to the output source in a single stream. 

Architecture:

                                 ____________                
                                /             \
                               16550           16550
                              /                 \
pixhawk <--14550--> mavproxy /                   \ mavproxy <--14552--> GC
                             \                   /
                              \                 /
                              16551            16552
                                \             /
                        mavros || this code__/    

    Pixhawk is sending messages via one instance of mavproxy master is 127.0.0.1:14550 to out at 127.0.0.1:16550
     and 127.0.0.1:16551 .
    Mavros is connected on 127.0.0.1:16551
    
    A second instance of mavproxy is launched with with 127.0.0.1:16550 and 127.0.0.1:16552 as master and out on 127.0.0.1:14552.
     Data from pixhawk on 16550 is directly relayed to 14552.
    
This code connects on 127.0.0.1:16552 and sends a single attitude message. It sets the source id as 1 which is same as
messages from pixhawk. When messages reach the GC it assumes both messages originate from a single source and sets the attitude
sent by this code at 2hz.
    
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


    def run(self):
        pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        rospy.init_node('commNode', anonymous=True)
        rate = rospy.Rate(2)  # 100hz

        while not rospy.is_shutdown():
            att_msg = AttitudeTarget()

            yaw = self.heading_data

            quat = self.to_quaternion(-0.0, 0.0, 90) # Set yaw to 90 then servo4 outputs 1500....need to figure out why

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
            self.autopilot.mav.attitude_send(0, 10, 20, 180,
                                             0, 0, 0)


            rate.sleep()


if __name__ == '__main__':
    rc = RadioComm()
    rc.run()
