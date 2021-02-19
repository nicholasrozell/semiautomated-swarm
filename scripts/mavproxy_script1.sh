#!/bin/bash
mavproxy.py --master=udp:127.0.0.1:14550 --out=udpout:127.0.0.1:16550 --out=udp:127.0.0.1:16551 --daemon
