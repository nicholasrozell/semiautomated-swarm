#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/r2/ros_ws/devel/setup.bash

sleep 2
#rosservice call /mavros/set_stream_rate 0 10 1
sleep 2
roslaunch swarm swarmnodes.launch
