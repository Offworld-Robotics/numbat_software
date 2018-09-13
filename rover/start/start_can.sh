#!/usr/bin/env bash

sudo slcand -o -c -s6 /dev/serial/by-id/usb-CANtact_CANtact_dev_00000000001A-if00 can0
sudo ifconfig can0 up
rosrun ros_can_nodes ros_can_nodes
