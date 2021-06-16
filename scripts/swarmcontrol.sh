#!/bin/bash
source /opt/ros/noetic/setup.bash

cd ~/semiautomated-swarm/build
./swarmcontrol > ~/semiautomated-swarm/logs/"$(date +"swarmcontorl__%FT%T").log"
