#!/bin/bash
source /opt/ros/noetic/setup.bash

cd $HOME/semiautomated-swarm/build
./swarmcontrol > $HOME/semiautomated-swarm/logs/"$(date +"swarmcontorl__%FT%T").log"
