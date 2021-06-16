#!/bin/bash

source /opt/ros/noetic/setup.bash
cd $HOME/semiautomated-swarm/python
python3 -u flight_test.py >> $HOME/semiautomated-swarm/logs/"$(date +"%FT%T__flightest").log"
