#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/r2/ros_ws/devel/setup.bash

#cd /home/sparky/semiautomated-swarm/python
#python3 -u flight_test.py >> /home/sparky/semiautomated-swarm/logs/"$(date +"flighttest__%FT%T").log"

sleep 2
roslaunch swarm planning.launch
