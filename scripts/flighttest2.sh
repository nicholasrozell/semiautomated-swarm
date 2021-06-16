#!/bin/bash

source /opt/ros/noetic/setup.bash
cd $HOME/semiautomated-swarm/python
python3 -u flight_test2.py > $HOME/semiautomated-swarm/logs/"$(date +"flighttest2__%FT%T").log"
