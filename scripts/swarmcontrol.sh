#!/bin/bash
source /opt/ros/noetic/setup.bash

cd /home/$USER/semiautomated-swarm/build
./swarmcontrol > /home/$USER/semiautomated-swarm/logs/"$(date +"swarmcontorl__%FT%T").log"
