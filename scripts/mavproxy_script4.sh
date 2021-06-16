#!/bin/bash
/usr/local/bin/mavproxy.py --master=/dev/ttyACM1,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --state-basedir=/home/$USER/semiautomated-swarm/logs/ --daemon
