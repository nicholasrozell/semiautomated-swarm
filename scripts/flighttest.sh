#!/bin/bash

source /opt/ros/noetic/setup.bash
cd /home/sparky/semiautomated-swarm/python
python3 -u flight_test.py >> /home/sparky/semiautomated-swarm/logs/"$(date +"%FT%T__flightest").log"
