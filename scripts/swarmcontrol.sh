#!/bin/bash
source /opt/ros/noetic/setup.bash

cd /home/sparky/semiautomated-swarm/build
./swarmcontrol > /home/sparky/semiautomated-swarm/logs/"$(date +"swarmcontorl__%FT%T").log"
