#!/bin/bash

source /opt/ros/noetic/setup.bash
cd /home/$USER/semiautomated-swarm/python
python3 -u flight_test2.py > /home/$USER/semiautomated-swarm/logs/"$(date +"flighttest2__%FT%T").log"
