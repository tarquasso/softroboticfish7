#include "mbed.h"
#include <ros.h>
#include <fish_msgs/joystick_in.h>

ros::NodeHandle cmd_in;
PwmOut led_freq(LED1);
PwmOut led_speed(LED2);
PwmOut led_yaw(LED4);
PwmOut led_depth(LED3);


void setLEDs(const fish_msgs::joystick_in& cmd_msg) {
	led_yaw = float((cmd_msg.yaw_ctrl + 128.0)/256.0);
	led_depth = float((cmd_msg.depth_ctrl + 128.0)/256.0);
	led_speed = float((cmd_msg.speed_ctrl + 128.0)/256.0);
	led_freq = float((cmd_msg.freq_ctrl + 128.0)/256.0);

	if (cmd_msg.yaw_ctrl > 0) {
		led_yaw = 1;
	} else {
		led_yaw = 0;
	}
	
	if (cmd_msg.depth_ctrl > 0) {
		led_depth = 1;
	} else {
		led_depth = 0;
	}
	if (cmd_msg.speed_ctrl > 0) {
		led_speed = 1;
	} else {
		led_speed = 0;
	}
}


ros::Subscriber<fish_msgs::joystick_in> serial_sub("to_serial", &setLEDs);

int main() {
	cmd_in.initNode();
	cmd_in.subscribe(serial_sub);

	while (1) {
		cmd_in.spinOnce();
		wait_ms(1);
	}
}
