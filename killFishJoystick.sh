echo "-- Killing fish joystick process"
sudo ps -ef | grep "fish/python/bluetooth_joystick/FishJoystickController.py" | awk '{print $2}' | xargs sudo kill


