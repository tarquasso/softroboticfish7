#!/usr/bin/env bash
echo "Setting up PYTHONPATH"
export PYTHONPATH=/home/ubuntu/softroboticfish6/fish/pi/ros/catkin_ws/src:$PYTHONPATH
echo "Activating development"
source ~/softroboticfish6/fish/pi/ros/catkin_ws/devel/setup.bash
echo "Setup ROS_HOSTNAME"
export ROS_HOSTNAME=$HOSTNAME.local
export FISHTOWN_ROOT=$HOME/softroboticfish6/fish/pi/ros/

