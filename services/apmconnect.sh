#!/bin/bash
source /opt/ros/noetic/setup.bash

availPorts=$(ls /dev | grep -E 'ttyACM')
echo "Avail Ports are " $availPorts
apmSerial=$(echo $availPorts | head -n1 | awk '{print $2;}') 
echo "Selected Port is "$apmSerial
roslaunch mavros apm.launch fcu_url:=/dev/$apmSerial:57600 tgt_system:=1
#roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:57600 tgt_system:=1
