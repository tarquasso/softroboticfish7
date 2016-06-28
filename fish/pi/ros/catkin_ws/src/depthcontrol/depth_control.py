import rospy
import numpy as np
from fish_msgs.msg import DepthTestMsg      # DepthTestMsg
from fish_msgs.msg import mbedStatusMsg      # mbedStatusMsg
from sensor_msgs.msg import Joy
import time


l2_button = 8
r2_button = 9
l1_button = 10
r1_button = 11
tri_button = 12
x_button = 14

l_stick_y_axis = 1

mode0_button = l1_button
mode1_button = r1_button
mode2_button = r2_button

tick_up_button = tri_button
tick_down_button = x_button

vel_ctrl_axis = l_stick_y_axis


class DepthJoy(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None

        self.pub = rospy.Publisher("", DepthTestMsg, queue_size = 1)

        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size = 1)

        self.sub_mbed = rospy.Subscriber("", mbedStatusMsg, self.cbMbed, queue_size = 1)
        self.mode_map = {3: "depth", 1 "voltage", 2: "position"}
        self.desired_number = {"depth": 0, "voltage": 0, "position": 0}
        self.mode = 1

        self.axis_gain_dict = {"depth": -1, "voltage": 1, "position": 1}
        self.tick_gain_dict = {"depth": -0.05, "voltage": 0.05, "position": 0.05}

        self.minmax_dict = {"depth": (0, 1), "volage": (0, 1), "position": (0,1)}

        self.gyro_axes = [23, 24, 25]
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
        if self.Joy != None:
            return {"axes": [(self.joy.axes[i] != joy_msg.axes[i]) and (i not in self.gyro_axes) for i in range(len(self.joy.axes))], "buttons": [self.joy.buttons[i] != joy_msg.buttons[i] for i in range(len(self.joy.axes))]}
        return {"axes": [(i not in self.gyro_axes) for i in range(len(joy_msg.axes))], "buttons": [True for i in range(len(joy_msg.axes))]}

    def joy_updated(self, diff):
        for cat in diff.keys():
            for item in diff[cat]:
                if item:
                    return True
        return False

    def normalize_axis(value, neg):
        if neg:
            return -(value/32768.0)
        return value/32768.0

    def clip(val, minmax):
        return min(max(val, minmax[0]), minmax[1])


    def do_incr(self, incr):
        increase = self.tick_gain_dict[self.mode_map[self.mode]] * incr
        self.desired_number[self.mode_map[self.mode]] = clip(increase + self.desired_number[self.mode_map[self.mode]], self.minmax_dict[mode_map[self.mode]])

    def cbJoy(self, joy_msg):
        diff = self.changed_axes_buttons(joy_msg)
        mbed_msg = DepthTestMsg()
        modifier = 0

        if self.joy_updated(diff):
            self.joy = joy_msg
            if diff["buttons"][mode0_button] and joy_msg["buttons"][mode0_button]:
                self.mode = 0
                # update msg with new mode
            if diff["buttons"][mode1_button] and joy_msg["buttons"][mode1_button]:
                self.mode = 1
            if diff["buttons"][mode2_button] and joy_msg["buttons"][mode1_button]:
                self.mode = 2
            if diff["buttons"][tick_up_button] and joy_msg["buttons"][tick_up_button]:
                do_incr(1)
            if diff["buttons"][tick_down_button] and joy_msg["buttons"][tick_down_button]:
                do_incr(-1)

        mbed_msg.mode = self.mode
        mbed_msg.value = self.desired_number[self.mode_map[self.mode]] + (self.axis_gain_dict[self.mode_map[self.mode]] * normalize_axis(joy_msg["axes"][vel_ctrl_axis], True))
        self.pub.publish(mbed_msg)

    def cbMbed(self, mbed_msg):
        rospy.loginfo("mode: %s, value: %s", mode_map[mbed_msg.mode], mbed_msg.value)




if __name__ == "__main__":
    rospy.init_node("depth_control_pi", anonymous=False)
    depth_control_pi = DepthJoy()
    rospy.spin()
