sleep 5
echo "- Startup script"
sudo /home/pi/fish/mbed/remountMbed.sh
#sudo python /home/pi/fish/mbed/resetMbed.py
echo "-- Starting video and low battery monitor"
sudo python /home/pi/fish/python/rx_controller.py &
echo "-- Starting serial monitor to record data"
cd /home/pi/fish/mbed
#sudo python /home/pi/fish/mbed/runProgram.py startup=1 print=0 file='data.wp' &
sudo /home/pi/fish/startup_rx_serial.sh &
echo "- Startup script complete"

