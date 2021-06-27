#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/r2/ros_ws/devel/setup.bash

sleep 2
#osservice call /mavros/set_stream_rate 0 10 1
sleep 10
roslaunch swarm swarmnodes.launch
