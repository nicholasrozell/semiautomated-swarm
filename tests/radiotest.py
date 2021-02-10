from pymavlink import mavutil
import time
import rospy
from mavros_msgs.msg import *
import struct
import socket


def convert_to_bytes(msg):
    """
    Re-builds the MAVLink byte stream from mavros_msgs/Mavlink messages.

    Support both v1.0 and v2.0.
    """
    payload_octets = len(msg.payload64)
    if payload_octets < msg.len / 8:
        raise ValueError("Specified payload length is bigger than actual payload64")

    if msg.magic == Mavlink.MAVLINK_V10:
        msg_len = 6 + msg.len  # header + payload length
        msgdata = bytearray(
            struct.pack(
                '<BBBBBB%dQ' % payload_octets,
                msg.magic, msg.len, msg.seq, msg.sysid, msg.compid, msg.msgid,
                *msg.payload64))
    else:  # MAVLINK_V20
        msg_len = 10 + msg.len  # header + payload length
        msgdata = bytearray(
            struct.pack(
                '<BBBBBBBBBB%dQ' % payload_octets,
                msg.magic, msg.len, msg.incompat_flags, msg.compat_flags, msg.seq,
                msg.sysid, msg.compid,
                msg.msgid & 0xff, (msg.msgid >> 8) & 0xff, (msg.msgid >> 16) & 0xff,
                *msg.payload64))

    if payload_octets != msg.len / 8:
        # message is shorter than payload octets
        msgdata = msgdata[:msg_len]

    # finalize
    msgdata += struct.pack('<H', msg.checksum)

    if msg.magic == Mavlink.MAVLINK_V20:
        msgdata += bytearray(msg.signature)

    return msgdata

class RadioComm:
    def callback(self, data):

        msg = convert_to_bytes(data)
        decoded = self.autopilot.mav.decode(msg)
        print(data.msgid)
        #if data.msgid == 0:
        #    print("MAVLINK_MSG_ID_HEARTBEAT")


        self.mavproxy_sock.sendto(msg, self.address)

    def state_test(self, data):
        print(data)
        #msg = convert_to_bytes(data)
        #decoded = self.autopilot.mav.decode(msg)
        #print(decoded)
        #self.mavproxy_sock.sendto(msg, self.address)

    def listener(self):
        rospy.init_node('commnode')
        rospy.Subscriber("/pixhawk/data",  Mavlink, self.callback)
        # spin() simply keeps python from exiting until this node is stopped

        self.autopilot = mavutil.mavlink_connection('udpout:localhost:14552')
        HOST = '127.0.0.1'
        mavproxy_port = 14552
        self.address = (HOST, mavproxy_port)
        self.mavproxy_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.spin()

if __name__ == '__main__':
    rc = RadioComm()
    rc.listener()


'''
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpout:localhost:14552')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
#the_connection.wait_heartbeat()
#print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

while True:
    print("sending heartbeat")
    the_connection.mav.heartbeat_send(
        6,  # type
        8,  # autopilot
        192,  # base_mode
        0,  # custom_mode
        4,  # system_status
        3  # mavlink_version
    )
    time.sleep(1)
    break
'''