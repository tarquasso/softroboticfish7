sleep 5
echo "- Startup script"
sudo /home/pi/fish/mbed/remountMbed.sh
#sudo python /home/pi/fish/mbed/resetMbed.py
echo "-- Starting video and low battery monitor"
sudo python /home/pi/fish/python/rx_controller.py &
#echo "-- Starting serial monitor to record data"
#cd /home/pi/fish/mbed
#sudo /home/pi/fish/startup_rx_serial.sh &
echo "-- Starting Joystick Controller"
sudo python /home/pi/fish/python/bluetooth_joystick/FishJoystickController.py &
sleep 5
sudo /home/pi/fish/killFishJoystick.sh 
sleep 3
sudo python /home/pi/fish/python/bluetooth_joystick/FishJoystickController.py &
echo "- Startup script complete"

