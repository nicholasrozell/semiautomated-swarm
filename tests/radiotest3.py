import rospy
from mavros_msgs.msg import *

'''
All the messages that are output

30: ATTITUDE
33: GLOBAL_POSITION_INT
1: SYS_STATUS
125: POWER_STATUS
62: NAV_CONTROLLER_OUTPUT
42: MISSION_CURRENT
74: VFR_HUD
36: SERVO_OUTPUT_RAW
65: RC_CHANNELS
27: RAW_IMU
116: SCALED_IMU2
129: SCALED_IMU3
29: SCALED_PRESSURE
0: HEARTBEAT
'''




class RadioComm:
    def __init__(self):
        pass

    def sys_status_cb(self, data):
        pass

