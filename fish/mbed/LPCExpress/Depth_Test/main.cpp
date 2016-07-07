#include <MS5837.h>
#include "mbed.h"
#include "BTU/btu_PID_lite.h"
#include <ros.h>
#include <fish_msgs/DepthTestMsg.h>
#include <fish_msgs/mbedStatusMsg.h>

#define PIN_IMU_SDA p28
#define PIN_IMU_SCL p27

#define rosTopicName "joy_control"

//DepthTestMsg
// Mode Int8
// Value Float32

#define TIMESTEP 0.05
#define DEPTH_MODE 3
#define VEL_MODE 1
#define POS_MODE 2
#define MIN_DEPTH 0
#define MAX_DEPTH 5.0
#define MIN_VEL -10.0
#define MAX_VEL 10.0
#define MIN_POS -91.0
#define MAX_POS 91.0


ros::NodeHandle nh;

fish_msgs::DepthTestMsg ctrl_msg;
fish_msgs::mbedStatusMsg stat_msg;
ros::Publisher statusPub("mbed_stat", &stat_msg);

DigitalOut modeHighOrder(LED1);
DigitalOut modeLowOrder(LED2);
PwmOut command(LED3);
PwmOut pressure(LED4);

float max_pressure = 0;
float min_pressure = -1;
float curDepth = 0;

int mode = VELOCITY_CTRL_MODE;
float desiredNumber = 0;
float mode_min = VEL_MIN;
float mode_max = VEL_MAX;

BTU btu = BTU();

void commandCb(const fish_msgs::DepthTestMsg& ctrl_msg) {
  mode = ctrl_msg.mode;
  desiredNumber = ctrl_msg.value;
  switch (mode) {
  case DEPTH_MODE:
    mode_min = DEPTH_MIN;
    mode_max = DEPTH_MAX;
    break;
 case VEL_MODE:
    mode_min = VEL_MIN;
    mode_max = VEL_MAX;
    break;
 case POS_MODE:
    mode_min = POS_MIN;
    mode_max = POS_MAX;
    break;
  }
}

ros::Subscriber<fish_msgs::DepthTestMsg> cmdSub("joy_control", &commandCb);

void displayMode() {
  modeHighOrder = mode / 2;
  modeLowOrder = mode % 2;
}

void displayPressure() {
  float curDepth = btu.getPressure();
  if (min_pressure == -1) {
    min_pressure = curDepth;
  }
  if (curDepth > max_pressure) {
    max_pressure = curDepth;
  }
  if (curDepth < min_pressure) {
    min_pressure = curDepth;
  }
  if(max_pressure != min_pressure) {
    pressure = (curDepth - min_pressure)/(max_pressure - min_pressure);
  }
}

void displayCommand() {
  // LED3
  command = (desiredNumber >= 0) ? desiredNumber : (-1*desiredNumber);
}

void runDisplay() {
  displayMode();
  displayPressure();
  displayCommand();
}

float scaleNumber(float desired) {
	if (mode == DEPTH_CTRL_MODE) {
		return desired * mode_max;
	}

	return (((desired + 1) * (mode_max - mode_min))/2) + (mode_min);

}

void initController() {
  btu.init();
}

void runController() {
  btu.updateAndRunCycle(mode, scaleNumber(desiredNumber));
}

void curateStatusMsg() {
  stat_msg.mode = mode;
  stat_msg.value = desiredNumber;
}

void loopFn() {
  runController();
}

int main() {
  nh.initNode();
  nh.subscribe(cmdSub);
  nh.advertise(statusPub);
  initController();
  Ticker timer;
  timer.attach(loopFn, TIMESTEP);
  while(1) {
    nh.spinOnce();
    // curateStatusMsg();
    stat_msg.mode = mode;
    stat_msg.value = desiredNumber;
    statusPub.publish( &stat_msg );
    runDisplay();
    wait(TIMESTEP);
  }

}
