#! /usr/bin/python

import rospy
import numpy as np
from fish_msgs.msg import FishControlMsg      # DepthTestMsg
from fish_msgs.msg import mbedDataMsg      # mbedStatusMsg
from sensor_msgs.msg import Joy
import time


l2_button = 8
r2_button = 9
l1_button = 10
r1_button = 11
tri_button = 12
x_button = 14
circle_button = 13
square_button = 15
dpad_right=5
dpad_left=7
dpad_up=4
dpad_down=6

l_stick_x_axis = 0
l_stick_y_axis = 1
r_stick_x_axis = 2
r_stick_y_axis = 3

mode1_button = l1_button
mode2_button = r1_button
mode3_button = l2_button
mode4_button = r2_button

depth_up_trim = tri_button
depth_down_trim = x_button

thrust_up_trim = dpad_up
thrust_down_trim = dpad_down

yaw_left_trim = dpad_left
yaw_right_trim = dpad_right

valve_spd_up_trim = circle_button
valve_spd_down_trim = square_button


thrust_ctrl_axis = l_stick_y_axis
depth_ctrl_axis = r_stick_y_axis
yaw_ctrl_axis = r_stick_x_axis



class FishJoy(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None

        self.pub = rospy.Publisher("joy_control", FishCtrlMsg, queue_size = 100)

        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size = 1)

        self.sub_mbed = rospy.Subscriber("mbed_stat", mbedStatusMsg, self.cbMbed, queue_size = 100)
        self.mode_map = {4: "depth", 2: "velocity", 3: "position", 1: "voltage"}
        self.desired_number = {"depth": 0, "velocity": 0, "position": 0, "voltage": 0, "yaw": 0, "thrust": 0.75, "valve": 0.2}
        self.mode = 2
        self.value = 0

        self.axis_gain_dict = {"depth": -1, "velocity": 1, "position": 1, "voltage": 1}
        self.tick_gain_dict = {"depth": -0.05, "velocity": 0.05, "position": 0.05, "voltage": 0.05, "yaw": 0.05, "thrust": 0.01, "valve": 0.05}

        self.minmax_dict = {"depth": (0, 5), "velocity": (-1, 1), "position": (0,1), "voltage": (-1,1), "thrust": (0.75, 1.0), "valve": (0, 1)}

        self.bad_axes = [23, 24, 25]
        self.mbed_msg = FishCtrlMsg()
        self.updateMsg()
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

    def do_incr(self, key, incr):
        param = key
        if param == "depth":
            param = self.mode_map[self.mode]
        increase = self.tick_gain_dict[param] * incr
        self.desired_number[param] = self.clip(increase + self.desired_number[param], self.minmax_dict[param])

    def updateMsg(self):
        self.mbed_msg = FishCtrlMsg()
        self.mbed_msg.mode = self.mode
        self.mbed_msg.dvalue = self.clip(self.desired_number[self.mode_map[self.mode]] + (self.axis_gain_dict[self.mode_map[self.mode]] * self.normalize_axis(joy_msg.axes[depth_ctrl_axis])),  self.minmax_dict[self.mode_map[self.mode]])

        self.mbed_msg.thrust = self.clip(self.desired_number["thrust"] + (self.axis_gain_dict["thrust"] * self.normalize_axis(joy_msg.axes[thrust_control_axis])), self.minmax_dict["thrust"])

        self.mbed_msg.yaw = self.clip(self.desired_number["yaw"] + (self.axis_gain_dict["yaw"] * self.normalize_axis(joy_msg.axes[yaw_control_axis])), self.minmax_dict["yaw"])

        self.mbed_msg.valve = self.clip(self.desired_number["valve"] + (self.axis_gain_dict["valve"] * self.normalize_axis(joy_msg.axes[valve_control_axis])), self.minmax_dict["valve"])


    def updateStats(self, joy_msg, diff):
        if not self.joy_updated(diff):
            return False

        self.joy = joy_msg
        sendMsg = False
        if diff["buttons"][mode1_button] and joy_msg.buttons[mode1_button]:
            self.mode = 1
            sendMsg = True
            # update msg with new mode
        if diff["buttons"][mode2_button] and joy_msg.buttons[mode2_button]:
            self.mode = 2
            sendMsg = True
        if diff["buttons"][mode3_button] and joy_msg.buttons[mode3_button]:
            self.mode = 3
            sendMsg = True
        if diff["buttons"][mode4_button] and joy_msg.buttons[mode4_button]:
            self.mode = 4
            sendMsg = True
        if diff["buttons"][depth_up_trim] and joy_msg.buttons[depth_up_trim]:
            self.do_incr("depth",1)
            sendMsg = True
        if diff["buttons"][depth_down_trim] and joy_msg.buttons[depth_down_trim]:
            self.do_incr("depth",-1)
            sendMsg = True
        if diff["buttons"][yaw_left_trim] and joy_msg.buttons[yaw_left_trim]:
            self.do_incr("yaw",1)
            sendMsg = True
        if diff["buttons"][yaw_right_trim] and joy_msg.buttons[yaw_right_trim]:
            self.do_incr("yaw",-1)
            sendMsg = True
        if diff["buttons"][thrust_up_trim] and joy_msg.buttons[thrust_up_trim]:
            self.do_incr("thrust",1)
            sendMsg = True
        if diff["buttons"][thrust_down_trim] and joy_msg.buttons[thrust_down_trim]:
            self.do_incr("thrust",-1)
            sendMsg = True
        if diff["buttons"][valve_spd_up_trim] and joy_msg.buttons[valve_spd_up_trim]:
            self.do_incr("valve", 1)
            sendMsg = True
        if diff["buttons"][valve_spd_down_trim] and joy_msg.buttons[valve_spd_down_trim]:
            self.do_incr("valve", -1)
            sendMsg = True

        if diff["axes"][depth_ctrl_axis] or diff["axes"][yaw_ctrl_axis] or diff["axes"][thrust_ctrl_axis]:
            sendMsg = True
        return sendMsg

    def cbJoy(self, joy_msg):
        diff = self.changed_axes_buttons(joy_msg)
 #       rospy.loginfo("%s", diff)

            # mbed_msg.mode = self.mode
            # self.value = self.clip(self.desired_number[self.mode_map[self.mode]] + (self.axis_gain_dict[self.mode_map[self.mode]] * self.normalize_axis(joy_msg.axes[vel_ctrl_axis])),  self.minmax_dict[self.mode_map[self.mode]])
            # mbed_msg.value = self.value
        sendMsg = self.updateStats(joy_msg, diff)
        if sendMsg:
            self.updateMsg()
        #     rospy.loginfo("PI mode: %s, value: %s", self.mode_map[mbed_msg.mode], mbed_msg.value)
                # self.pub.publish(mbed_msg)

    def cbMbed(self, mbed_msg):
        rospy.loginfo("MBED mode: %s, dvalue: %s, thrust: %s, yaw: %s, valve_spd: %s", self.mode_map[mbed_msg.mode], mbed_msg.dvalue, mbed_msg.thrust, mbed_msg.yaw, mbed_msg.valve)


    def sendStat(self):
        self.pub.publish(self.mbed_msg);



if __name__ == "__main__":
    rospy.init_node("fish_control_pi", anonymous=False)
    depth_control_pi = FishJoy()
    while not rospy.is_shutdown():
    # while True:
        fish_control_pi.sendStat()
        time.sleep(0.01)
    rospy.spin()
