sleep 5
echo "**** Startup script"

#sudo /home/pi/gitRepo/fish/pi/remountMbed.sh
sudo python /home/pi/softroboticfish7/fish/pi/resetMbed.py

echo "-- Starting low battery monitor and possibly video"
sudo python /home/pi/softroboticfish7/fish/pi/rx_controller.py &

# SERIAL LOGGING (streaming data from mbed)
echo  "-- Starting serial monitor to record streaming mbed data"
#sudo /home/pi/gitRepo/fish/pi/startup_rx_serialLogging.sh &

# BLUETOOTH JOYSTICK
echo "-- Starting Joystick Controller"
# sleep 5
# sudo python /home/pi/softroboticfish7/fish/pi/bluetooth_joystick/BluetoothJoystickController.py &
# sleep 5
# sudo python /home/pi/softroboticfish7/fish/pi/resetMbed.py
echo "- Startup script complete"

exit 0
