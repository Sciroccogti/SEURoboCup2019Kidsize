#!/bin/bash
catkin_make --pkg simulation
catkin_make --pkg game_ctrl
catkin_make
ws=`pwd`
echo "source $ws/devel/setup.bash" >> ~/.bashrc
echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
source ~/.bashrc