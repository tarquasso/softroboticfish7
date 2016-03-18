#!/bin/sh

echo "*** CONTROLLER STARTUP SCRIPT ***"

echo "--- killing existing fish controller processes"
sudo ps -ef | grep "AcousticJoystickController.py" | awk '{print $2}' | xargs sudo kill

echo "--- starting transmitter joystick control"
sudo python /home/pi/fish/controller/AcousticJoystickController.py &
sleep 2
echo "--- done with controller startup"
