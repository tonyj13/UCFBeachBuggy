#!/bin/bash

# Tony Jimogaon
# Senior Design 2 2018
# Group 1

# ==========================
#	Script to run GPS of buggy
# ==========================

sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket
sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock
rosrun gpsd_client gpsd_client _host:=localhost &
#./gps.py
