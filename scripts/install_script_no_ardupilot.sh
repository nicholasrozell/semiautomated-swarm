#!/bin/bash

# Update the libraries
sudo apt update

sleep 1s

# Upgrade the libraries
sudo apt upgrade

# install essentials
sudo apt install build-essential git openssh-server -y

printf "Installing ROS noetic"
sleep 1s

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sleep 1s

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

sleep 1s
sudo apt install ros-noetic-ros-base -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sleep 2s
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

sudo rosdep init
rosdep update

printf "COMPLETED ROS NOETIC INSTALL"

sleep 2s

sudo apt install ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-mavros-extras -y

sleep 1s
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

printf "INSTALLED MAVROS"

# Install mavproxy to root
sudo pip3 install MAVProxy
