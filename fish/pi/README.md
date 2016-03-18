***** SETUP *****
-- Copy contents of rc.local to your /etc/rc.local and make it executable ("sudo chmod +x /etc/rc.local")
-- Make all scripts in this folder executable ("sudo chmod +x /home/pi/gitRepo/fish/pi/*.sh")

===== rx_controller.py =====
  -- This will launch a low battery monitor that shuts down the Pi if a low battery is detected.
  -- This will also start video recording (segmented into 1-minute videos with pictures in between) if a camera is present.
  -- It will try to launch rx_controller_camera and if that fails will launch rx_controller_noCamera instead.

==== bluetooth_joystick =====
  -- Run BluetoothJoystickController.py to start reading from the bluetooth gamepad and streaming the state to the mbed via serial.
  -- killBluetoothJoystick.sh will terminate the BluetoothJoystickController.py process

===== viewFishProcesses.sh and killFishProcesses.sh =====
-- view or kill python and script processes related to the fish 

===== reboot.sh and shutdown.sh =====
-- Unmount the mbed and the reboot or shutdown

===== startup_rx.sh =====
-- Mounts the mbed
-- Starts rx_controller.py
-- Optionally starts serial monitor to log streaming data from the mbed (uncomment the lines launching startup_rx_serialLogging.sh to enable this)
-- Optionally starts the bluetooth joystick controller (uncomment the lines launching BluetoothJoystickController.py to enable this)

===== rc.local =====
-- Copy these contents into your /etc/rc.local
-- Make sure it's executable
-- Make sure startup*.sh scripts are executable

===== mountMbed, unmountMbed, remountMbed =====
-- will do the appropriate actions to mount/unmount mbed to /media/MBED

===== programMbed =====
-- will delete any bin files in /media/MBED and copy the bin file supplied as an argument to this script
  