import socket
import time
import SwarmComm.mavlink_apm as apmd

HOST = '127.0.0.1'
mavproxy_port = 14550
mavproxy_sock = socket.socket (socket.AF_INET,socket.SOCK_DGRAM)
mavproxy_sock.bind((HOST,mavproxy_port))
mav_obj = apmd.MAVLink (mavproxy_sock)

while True:
    data_from_mavproxy,address_of_mavproxy = mavproxy_sock.recvfrom (1024)
    decoded_message = mav_obj.decode(data_from_mavproxy)

    print(decoded_message)
    time.sleep(0.01)