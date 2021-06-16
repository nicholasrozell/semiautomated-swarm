#!/bin/bash

source /opt/ros/noetic/setup.bash
cd /home/$USER/semiautomated-swarm/python
python3 -u flight_test.py >> /home/$USER/semiautomated-swarm/logs/"$(date +"%FT%T__flightest").log"
