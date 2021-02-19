#!/bin/bash

cd ../services
sudo cp *.service /etc/systemd/system

sleep 1s
sudo systemctl daemon-reload

sleep 1s
sudo systemctl start Mavproxy_Pix.service
printf "Starting Mavproxy link from Pixhawk\n"

sleep 1s
sudo systemctl start Mavproxy_Comm.service
printf "Starting Mavproxy link from Comm device\n"

sleep 1s
sudo systemctl start MavROS.service
printf "Starting Mavros service\n"
