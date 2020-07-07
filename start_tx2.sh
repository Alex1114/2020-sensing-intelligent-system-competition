#! /bin/bash

ETH0_IP=$(ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)
WLAN0_IP=$(ip addr show wlan0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)

source /home/sis/sis_competition_template/environment.sh
export ROS_IP=10.42.0.1

roslaunch teleop_twist_joy teleop.launch eth0:=$ETH0_IP wlan0:=$WLAN0_IP joy_dev:=/dev/input/js0
