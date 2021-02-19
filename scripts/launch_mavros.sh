#!/bin/bash

source /opt/ros/noetic/setup.bash
roslaunch mavros apm.launch fcu_url:=udp://:16551@