#! /usr/bin/python

import rospy
import numpy as np
from fish_msgs.msg import PumpTestMsg      # DepthTestMsg
from fish_msgs.msg import mbedPumpStatusMsg      # mbedStatusMsg
from sensor_msgs.msg import Joy
import time


l2_button = 8
r2_button = 9
l1_button = 10
r1_button = 11
tri_button = 12
x_button = 14

l_stick_y_axis = 1
r_stick_x_axis = 2
dpad_right=5
dpad_left=7

mode1_button = l1_button
mode2_button = r1_button

tick_up_button = tri_button
tick_down_button = x_button

yaw_right_button = dpad_right
yaw_left_button = dpad_left

freq_ctrl_axis = l_stick_y_axis
yaw_ctrl_axis = r_stick_x_axis


class PumpJoy(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None

        self.pub = rospy.Publisher("joy_control", PumpTestMsg, queue_size = 100)

        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size = 1)

        self.sub_mbed = rospy.Subscriber("mbed_stat", mbedPumpStatusMsg, self.cbMbed, queue_size = 100)
        self.mode_map = {1: "AM", 2: "FM"}
        self.desired_number = {"yaw": 0, "freq": 0.75}
        self.mode = 25
        self.freq_value = 0
        self.yaw_value = 0

        self.axis_gain_dict = {"yaw": -1, "freq": 0.25}
        self.tick_gain_dict = {"yaw": 0.05, "freq": 0.01}

        self.minmax_dict = {"yaw": (-1, 1), "freq": (0.75, 1)}

        self.bad_axes = [23, 24, 25]
        self.mbed_msg = PumpTestMsg()
        # Controller:
        # LStick: x0, -y1
        # RStick: x2, -y3
        # LDpad: b7, ? (projected a11)
        # RDpad: b5, a9
        # UDpad: b4, a8
        # DDpad: b6, a10
        # X: b14, a18
        # Tri: b12, a16
        # Square: b15, a19
        # Circle: b13, a17
        # L1: b10, a14
        # R1: b11, a15
        # L2: b8, a12
        # R2: b9, a13
        # Select b0
        # Start b3
        # PSbutton b16
        # L3 b1
        # R3 b2



    def changed_axes_buttons(self, joy_msg):
        if self.joy != None:
            return {"axes": [(self.joy.axes[i] != joy_msg.axes[i]) and (i not in self.bad_axes) for i in range(len(self.joy.axes))], "buttons": [self.joy.buttons[i] != joy_msg.buttons[i] for i in range(len(self.joy.buttons))]}
        return {"axes": [(i not in self.bad_axes) for i in range(len(joy_msg.axes))], "buttons": [True for i in range(len(joy_msg.buttons))]}

    def joy_updated(self, diff):
        for cat in diff.keys():
#            rospy.loginfo("diff: %s", diff[cat])
            for i in range(len(diff[cat])):
                if diff[cat][i]:
                    # rospy.loginfo("%s %s",i, diff[cat][i])
                    return True
        return False

    def normalize_axis(self, value):
        # rospy.loginfo("%s", value)
        return value

    def clip(self, val, minmax):
        return min(max(val, minmax[0]), minmax[1])


    def do_incr(self, param, incr):
        increase = self.tick_gain_dict[param] * incr
        self.desired_number[param] = self.clip(increase + self.desired_number[param], self.minmax_dict[param])

    def cbJoy(self, joy_msg):
        diff = self.changed_axes_buttons(joy_msg)
 #       rospy.loginfo("%s", diff)

        mbed_msg = PumpTestMsg()
        modifier = 0

        if self.joy_updated(diff):
            self.joy = joy_msg
            sendMsg = False
            if diff["buttons"][mode1_button] and joy_msg.buttons[mode1_button]:
                self.mode -= (1 if self.mode > 0 else 0)
                sendMsg = True
                # update msg with new mode
            if diff["buttons"][mode2_button] and joy_msg.buttons[mode2_button]:
                self.mode += (1 if self.mode < 100 else 0)
                sendMsg = True
            if diff["buttons"][tick_up_button] and joy_msg.buttons[tick_up_button]:
                self.do_incr("freq", 1)
                sendMsg = True
            if diff["buttons"][tick_down_button] and joy_msg.buttons[tick_down_button]:
                self.do_incr("freq", -1)
                sendMsg = True
            if diff["buttons"][yaw_right_button] and joy_msg.buttons[yaw_right_button]:
                self.do_incr("yaw", 1)
            if diff["buttons"][yaw_left_button] and joy_msg.buttons[yaw_left_button]:
                self.do_incr("yaw", -1)
            if diff["axes"][freq_ctrl_axis] or diff["axes"][yaw_ctrl_axis]:
                sendMsg = True


            mbed_msg.mode = self.mode
            self.freq_value = self.clip(self.desired_number["freq"] + (self.axis_gain_dict["freq"] * self.normalize_axis(joy_msg.axes[freq_ctrl_axis])),  self.minmax_dict["freq"])
            self.yaw_value = self.clip(self.desired_number["yaw"] + (self.axis_gain_dict["yaw"] * self.normalize_axis(joy_msg.axes[yaw_ctrl_axis])),  self.minmax_dict["yaw"])

            mbed_msg.freq = self.freq_value
            mbed_msg.yaw = self.yaw_value
            if sendMsg:
                rospy.loginfo("PI mode: %s, freq: %s, yaw: %s", mbed_msg.mode, mbed_msg.freq, mbed_msg.yaw)
                # self.pub.publish(mbed_msg)

    def cbMbed(self, mbed_msg):
        rospy.loginfo("MBED mode: %s, freq: %s, yaw: %s", mbed_msg.mode, mbed_msg.freq, mbed_msg.yaw)


    def sendStat(self):
        mbed_msg = PumpTestMsg()
        mbed_msg.mode = self.mode;
        mbed_msg.yaw = self.yaw_value;
        mbed_msg.freq = self.freq_value;
        self.pub.publish(mbed_msg);



if __name__ == "__main__":
    rospy.init_node("pump_control_pi", anonymous=False)
    pump_control_pi = PumpJoy()
    while not rospy.is_shutdown():
    # while True:
        pump_control_pi.sendStat()
        time.sleep(0.01)
    rospy.spin()
