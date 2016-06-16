#include "mbed.h"
#include <ros.h>
#include <fish_msgs/joystick_in.h>

ros::NodeHandle cmd_in;
PwmOut led_freq(LED1);
PwmOut led_speed(LED3);
PwmOut led_yaw(LED2);
PwmOut led_depth(LED4);

bool bcu_state = 0;
bool valve_state = 0;
bool prev_bcu_toggle = 0;
bool prev_valve_toggle = 0;


void setLEDs(const fish_msgs::joystick_in& cmd_msg) {

	// start/stop processes
	if (cmd_msg.bcu_toggle && !prev_bcu_toggle) {
		bcu_state = !bcu_state;
		if (bcu_state) { // start bcu
			led_depth = 1.0;
			wait(0.2);
			led_depth = 0.0;
			wait(0.2);
			led_depth = 1.0;
			wait(0.2);
			led_depth = 0.0;
		} else { // stop bcu
			led_depth = 1.0;
			wait(0.5);
			led_depth = 0.0;
		}
	}
	if (cmd_msg.valve_toggle && !prev_valve_toggle) {
		valve_state = !valve_state;
		if (valve_state) { // start pump with valve
			led_yaw = 1.0;
			led_freq = 1.0;
			led_speed = 1.0;
			wait(0.2);
			led_yaw = 0.0;
			led_freq = 0.0;
			led_speed = 0.0;
			wait(0.2);
			led_yaw = 1.0;
			led_freq = 1.0;
			led_speed = 1.0;
			wait(0.2);
			led_yaw = 0.0;
			led_freq = 0.0;
			led_speed = 0.0;
		} else { // stop pump with valve
			led_yaw = 1.0;
			led_freq = 1.0;
			led_speed = 1.0;
			wait(0.5);
			led_yaw = 0.0;
			led_freq = 0.0;
			led_speed = 0.0;
		}
	}


	// set LEDs for running processes
	// inputs range from -127 to 127
	if (bcu_state) {
		led_depth = float((cmd_msg.depth_ctrl + 127.0)/254.0);
	} else if (!cmd_msg.bcu_backwards && !cmd_msg.bcu_forwards) {
		led_depth = 0.0;
	}

	if (valve_state) {
		led_yaw = float((cmd_msg.yaw_ctrl + 127.0)/254.0);
		led_freq = float((cmd_msg.freq_ctrl + 127.0)/254.0);
		led_speed = float((cmd_msg.speed_ctrl + 127.0)/254.0); //thrust
	} else {
		led_yaw = 0.0;
		led_freq = 0.0;
		led_speed = 0.0;
	}



	if (cmd_msg.bcu_backwards) {
		if (bcu_state) {
			// bcu.stop()
			bcu_state = 0;
			led_depth = 1.0;
			wait(0.5);
			led_depth = 0.0;
			wait(0.5);
			led_depth = 0.25;
		} else {
			led_depth = 0.25;
		}
	}

	if (cmd_msg.bcu_forwards) {
		if (bcu_state) {
			// bcu.stop()
			bcu_state = 0;
			led_depth = 1.0;
			wait(0.5);
			led_depth = 0.0;
			wait(0.5);
			led_depth = 0.75;
		} else {
			led_depth = 0.75;
		}
	}	

	prev_bcu_toggle = cmd_msg.bcu_toggle;
	prev_valve_toggle = cmd_msg.valve_toggle;

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
