#!/bin/sh

echo "--- killing existing fish controller processes"
sudo ps -ef | grep "fish/python/new_tx/tx_controller.py" | awk '{print $2}' | xargs sudo kill

echo "--- starting transmitter joystick control"
sudo python /home/pi/fish/python/new_tx/tx_controller.py &
sleep 2
echo "--- done with startup"
