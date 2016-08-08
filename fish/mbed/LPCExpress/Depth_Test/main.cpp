#include <MS5837.h>
#include "mbed.h"
#include "BtuLinearServo/BtuLinearServo.h"
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
#define VOLT_MODE 1
#define DEPTH_MODE 4
#define VEL_MODE 2
#define POS_MODE 3
#define MIN_DEPTH 0
#define MAX_DEPTH 5.0
#define MIN_VEL -10.0
#define MAX_VEL 10.0
#define MIN_POS -91.0
#define MAX_POS 91.0

#define LOOPS_PER_STATUS 100

ros::NodeHandle nh;

fish_msgs::DepthTestMsg ctrl_msg;
fish_msgs::mbedStatusMsg stat_msg;
ros::Publisher statusPub("mbed_stat", &stat_msg);

DigitalOut modeHighOrder(LED1);
DigitalOut modeLowOrder(LED2);
PwmOut command(LED3);
PwmOut pressure(LED4);

float max_pressure = 1300;
float min_pressure = 1000;
float curDepth = 0;

int mode = POSITION_CTRL_MODE;
float desiredNumber = 0;
float mode_min = POS_MIN;
float mode_max = POS_MAX;

bool cycle = false;

BtuLinearServo btu = BtuLinearServo();

void commandCb(const fish_msgs::DepthTestMsg& ctrl_msg) {
  mode = ctrl_msg.mode;
  desiredNumber = ctrl_msg.value;
  switch (mode) {
  case DEPTH_CTRL_MODE:
    mode_min = DEPTH_MIN;
    mode_max = DEPTH_MAX;
    break;
 case VELOCITY_CTRL_MODE:
    mode_min = VEL_MIN;
    mode_max = VEL_MAX;
    break;
 case POSITION_CTRL_MODE:
    mode_min = POS_MIN;
    mode_max = POS_MAX;
    break;
  case VOLTAGE_CTRL_MODE:
    mode_min = VOLT_MIN;
    mode_max = VOLT_MAX;
  }
}

ros::Subscriber<fish_msgs::DepthTestMsg> cmdSub("joy_control", &commandCb);

void displayMode() {
  int dmode = mode -1;
  modeHighOrder = dmode / 2;
  modeLowOrder = dmode % 2;
}

void displayPressure() {
  float curDepth = btu.getPressure();
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
  //displayPressure();
  // displayCommand();
}

float scaleNumber(float desired) {
	if (mode == DEPTH_CTRL_MODE) {
		return desired * mode_max;
	} else if(mode == POSITION_CTRL_MODE) {
      return desired;
    }

	return (((desired + 1) * (mode_max - mode_min))/2) + (mode_min);

}

void initController() {
  btu.init();
  btu.setDryMode(false);
}

void runController() {
  btu.updateAndRunCycle(mode, scaleNumber(desiredNumber));
}

void curateStatusMsg() {
  stat_msg.mode = mode;
  stat_msg.value = desiredNumber;
}

void loopFn() {
  // runController();
  cycle = true;
}




int main() {
  nh.initNode();
  nh.subscribe(cmdSub);
  nh.advertise(statusPub);
  initController();
  Ticker timer;
  // timer.attach(loopFn, TIMESTEP);
  int counter=0;
  while(1) {
    nh.spinOnce();

    // if(cycle) {
      if(counter == 0) {
        // curateStatusMsg();
        // statusPub.publish( &stat_msg );
        if(command < 1) {
          command = 1;
        } else {
          command = 0;
        }
      }
      //stat_msg.mode = mode;
      //stat_msg.value = desiredNumber;
      counter = (counter + 1) % LOOPS_PER_STATUS;
      runController();
      curateStatusMsg();

      statusPub.publish(&stat_msg);

      runDisplay();
      cycle = false;
      // wait(TIMESTEP);
    // }
      wait(TIMESTEP);
  }

}
