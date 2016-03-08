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
#        self.last_pub_msg = None
#        self.last_pub_time = rospy.Time.now()

        # Setup Parameters
#        self.speed_gain = self.setupParam("~speed_gain", 1.0)
#        self.steer_gain = self.setupParam("~steer_gain", 1.0)

        # Publications
#        self.pub_wheels = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
	self.pub_toSerial = rospy.Publisher("to_serial", joystick_in, queue_size = 1)

        # Subscriptions
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        
        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
#        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
#        self.has_complained = False

#    def cbParamTimer(self,event):
#        self.speed_gain = rospy.get_param("~speed_gain", 1.0)
#        self.steer_gain = rospy.get_param("~steer_gain", 1.0)

#    def setupParam(self,param_name,default_value):
#        value = rospy.get_param(param_name,default_value)
#        rospy.set_param(param_name,value) #Write to parameter server for transparancy
#        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
#        return value

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
	
	cmd_out = joystick_in()	

	cmd_out.header.stamp = self.joy.header.stamp
	cmd_out.speed_ctrl = int(self.joy.axes[1] * 127)
	cmd_out.depth_ctrl = int(self.joy.axes[4] * 127)
	cmd_out.yaw_ctrl = int(self.joy.axes[3] * 127)
	
	self.pub_toSerial.publish(cmd_out)

#        speed = self.joy.axes[1] * self.speed_gain #Left stick V-axis. Up is positive
#        steering = self.joy.axes[3] * self.steer_gain
#        wheels_cmd_msg = WheelsCmdStamped()

        # Car Steering Mode
#        vel_left = (speed - steering)
#        vel_right = (speed + steering)
#        wheels_cmd_msg.header.stamp = self.joy.header.stamp
#        wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
#        wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)
#        rospy.loginfo("[%s] left %f, right %f" % (self.node_name,wheels_cmd_msg.vel_left,wheels_cmd_msg.vel_right))
#        self.pub_wheels.publish(wheels_cmd_msg)

if __name__ == "__main__":
    rospy.init_node("joy_cmds",anonymous=False)
    joy_cmds = JoyMapper()
    rospy.spin()
