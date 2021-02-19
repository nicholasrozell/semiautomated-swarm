#!/bin/bash
mavproxy.py --master=udp:127.0.0.1:14552 --out=udp:192.168.1.254:8990 --daemon
