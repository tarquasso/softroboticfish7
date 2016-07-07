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
#define VOLT_MODE 1
#define POS_MODE 2
#define MIN_DEPTH 0
#define MAX_DEPTH 1.0
#define MIN_VOLT 0
#define MAX_VOLT 1.0
#define MIN_POS 0
#define MAX_POS 1.0


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

int mode = 1;
float desiredNumber = 0;
float mode_min = MIN_VOLT;
float mode_max = MAX_VOLT;

BTU btu = BTU();

void commandCb(const fish_msgs::DepthTestMsg& ctrl_msg) {
  mode = ctrl_msg.mode;
  desiredNumber = ctrl_msg.value;
  switch (mode) {
  case DEPTH_MODE:
    mode_min = MIN_DEPTH;
    mode_max = MAX_DEPTH;
    break;
 case VOLT_MODE:
    mode_min = MIN_VOLT;
    mode_max = MAX_VOLT;
    break;
 case POS_MODE:
    mode_min = MIN_POS;
    mode_max = MAX_POS;
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
  command = (desiredNumber - mode_min)/(mode_max - mode_min);
}

void runDisplay() {
  displayMode();
  displayPressure();
  displayCommand();
}

void initController() {
  btu.init();
}

void runController() {
  btu.updateAndRunCycle(mode, desiredNumber);
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
