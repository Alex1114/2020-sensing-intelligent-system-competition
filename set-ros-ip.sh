#! /bin/bash


WLAN0_IP=$(ip addr show wlan0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)
export ROS_IP=$WLAN0_IP
echo ROS_IP=$WLAN0_IP
