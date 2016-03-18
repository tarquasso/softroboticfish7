***** SETUP *****
-- Copy contents of rc.local to your /etc/rc.local and make it executable ("sudo chmod +x /etc/rc.local")
-- Make all scripts in this folder executable ("sudo chmod +x /home/pi/gitRepo/controller/*.sh")


===== rc.local =====
-- Put these contents into your /etc/rc.local (and make sure it's executable)
-- Will call startup_tx.sh to start the transmitter controller

===== startup_tx.sh =====
-- This will kill any existing processes of AcousticJoystickController.py and then start AcousticJoystickController.py
-- This should be called on startup (see rc.local)
-- Make sure this is executable

===== AcousticJoystickController.py =====
-- This is for the acoustic transmitter that uses the SNES gamepad.
-- It will launch three processes to read state from the joystick, update the LEDs to reflect the state, and send desired state acoustically.
-- It will create a heartbeat that flashes an LED from blue to green if all is running well.
-- If anything breaks, the hearbeat will switch to alternating red/orange (if one of the subprocesses died), being solid red (if something went wrong in the main thread), or freezing in its blue/green heartbeat (if the main thread was terminated and not allowed to exit cleanly)

===== getVolume and setVolume =====
-- View or adjust the volume (for acoustic playback)
-- Be wary of damaging the hydrophones with loud signals