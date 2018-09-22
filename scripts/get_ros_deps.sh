#!/usr/bin/env bash

mkdir -p ~/ros_deps
cd ~/ros_deps
git clone https://github.com/bluesat/16_04_ros_deps.git src
cd src
git submodule init
git submodule update
cd ..
catkin_make -DGSTREAMER_VERSION_1_x=1  
# echo "source ~/ros_deps/devel/setup.bash" >> ~/.bashrc
source ~/ros_deps/devel/setup.bash
