#!/bin/bash
cd ../services
sudo cp *.service /etc/systemd/system

sleep 1s
sudo systemctl start Mavproxy_Pix.service

sleep 1s
sudo systemctl start Mavproxy_Comm.service

sleep 1s
sudo systemctl start MavROS@$1.service
