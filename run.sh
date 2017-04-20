#!/bin/sh

#roslaunch crazyflie_auto tracker_realsense_r200.launch
mv $HOME/sampling $HOME/sampling`date +%s`
mkdir $HOME/sampling
roslaunch crazyflie_auto takeoff_test.launch
