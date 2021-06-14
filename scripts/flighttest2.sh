#!/bin/bash

source /opt/ros/noetic/setup.bash
cd /home/sparky/semiautomated-swarm/python
python3 -u flight_test2.py > /home/sparky/semiautomated-swarm/logs/"$(date +"flighttest2__%FT%T").log"
