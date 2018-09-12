#!/usr/bin/env bash

source ~/ros_can_nodes/devel/setup.bash
sudo slcand -o -c -s6 /dev/serial/by-id/usb-CANtact_CANtact_dev_00000000001A-if00 can0;
sudo ifconfig can0 up
sleep 1
sudo ifconfig can0 txqueuelen 1000
rosrun ros_can_nodes ros_can_nodes 
