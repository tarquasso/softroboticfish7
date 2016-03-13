#!/usr/bin/env python
import rospy
import numpy as np
from fish_msgs.msg import joystick_in
from sensor_msgs.msg import Joy
import time

class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.joy = None

        # Setup Parameters
#        self.speed_gain = self.setupParam("~speed_gain", 1.0)
#        self.steer_gain = self.setupParam("~steer_gain", 1.0)

        # Publications
#        self.pub_wheels = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
	self.pub_toSerial = rospy.Publisher("to_serial", joystick_in, queue_size = 1)
#	cmd_init = joystick_in(rospy.Time.now(), 0,0,0,0)
#	self.pub_toSerial.publish(cmd_init)

        # Subscriptions
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()

    def publishControl(self):
	# relevant tutorial: http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
	# button mapping: Joystick (Logitech Gamepad F710) has 8 axes (X, Y, Z, Rx, Ry, Rz, Hat0X, Hat0Y)
	#       & 11 buttons (BtnX, BtnY, BtnTL, BtnTR, BtnTR2, BtnSelect, BtnThumbL, BtnThumbR, ?, ?, ?).

	# pressure/depth control - right stick up/down (+1.0 up to -1.0 down)
	# yaw control - right stick left/right (-1.0 right to +1.0 left)
	# speed control - left stick up/down (+1.0 up to -1.0 down)
	# frequency control - left stick right/left (-1.0 right to +1.0 left)
	
	cmd_out = joystick_in()	

	cmd_out.header.stamp = self.joy.header.stamp
	cmd_out.freq_ctrl = -1*int(self.joy.axes[0] * 127)
	cmd_out.speed_ctrl = int(self.joy.axes[1] * 127)
	cmd_out.yaw_ctrl = -1*int(self.joy.axes[3] * 127)
	cmd_out.depth_ctrl = int(self.joy.axes[4] * 127)

	
	self.pub_toSerial.publish(cmd_out)

#        speed = self.joy.axes[1] * self.speed_gain #Left stick V-axis. Up is positive
#        steering = self.joy.axes[3] * self.steer_gain
#        wheels_cmd_msg = WheelsCmdStamped()



if __name__ == "__main__":
    rospy.init_node("joy_cmds",anonymous=False)
    joy_cmds = JoyMapper()
    rospy.spin()
