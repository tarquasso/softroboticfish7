#!/usr/bin/env bash
echo "Activating ROS"
source /opt/ros/indigo/setup.bash
echo "Setting up PYTHONPATH"
export PYTHONPATH=/home/ubuntu/softroboticfish6/catkin_ws/src:$PYTHONPATH
echo "Activating development"
source ~/softroboticfish6/catkin_ws/devel/setup.bash
echo "Setup ROS_HOSTNAME"
export ROS_HOSTNAME=$HOSTNAME.local
export FISHTOWN_ROOT=$HOME/softroboticfish6

echo "Setting up GCC4MBED_DIR"
export GCC4MBED_DIR=~/gcc4mbed
export ROS_LIB_DIR=~/softroboticfish6/mbed/ros_lib
export GCC4BMED_DEPLOY='cp PROJECT.bin /media/MBED ; sync'
export PATH=~/gcc4mbed/gcc-arm-none-eabi/bin:$PATH

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
