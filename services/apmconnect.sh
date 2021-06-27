#!/bin/bash

source /opt/ros/noetic/setup.bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM1 tgt_system:=1