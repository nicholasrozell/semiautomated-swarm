#!/bin/bash

source /opt/ros/noetic/setup.bash
cd ~/semiautomated-swarm/python
python3 -u flight_test.py >> ~/semiautomated-swarm/logs/"$(date +"%FT%T__flightest").log"
