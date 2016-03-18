echo "-- Killing bluetooth joystick process"
sudo ps -ef | grep "BluetoothJoystickController.py" | awk '{print $2}' | xargs sudo kill &> /dev/null


