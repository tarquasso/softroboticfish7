#include <MS5837.h>
#include "mbed.h"
// #include "BtuLinearServo/BtuLinearServo.h"
#include <CyclicActuatorSync/CyclicActuatorSync.h>
#include <ros.h>
#include <fish_msgs/PumpTestMsg.h>
#include <fish_msgs/mbedPumpStatusMsg.h>

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


#define LOOPS_PER_STATUS 100

ros::NodeHandle nh;

fish_msgs::PumpTestMsg ctrl_msg;
fish_msgs::mbedPumpStatusMsg stat_msg;
ros::Publisher statusPub("mbed_stat", &stat_msg);

DigitalOut modeHighOrder(LED1);
DigitalOut modeLowOrder(LED2);
PwmOut command(LED3);
PwmOut pressure(LED4);

float max_pressure = 1300;
float min_pressure = 1000;
float curDepth = 0;

int mode = 2;
float desiredNumber = 0;
// float mode_min = POS_MIN;
// float mode_max = POS_MAX;
float yaw = 0;
float freq = 0;

bool cycle = false;

// BtuLinearServo btu = BtuLinearServo();
CyclicActuatorSync cac = CyclicActuatorSync();

void commandCb(const fish_msgs::PumpTestMsg& ctrl_msg) {
  mode = ctrl_msg.mode;
  freq = ctrl_msg.freq;
  yaw = ctrl_msg.yaw;
}

ros::Subscriber<fish_msgs::PumpTestMsg> cmdSub("joy_control", &commandCb);

void displayMode() {
  int dmode = mode - 1;
  modeHighOrder = dmode / 2;
  modeLowOrder = dmode % 2;
}

// void displayPressure() {
//   float curDepth = btu.getPressure();
//   if (curDepth > max_pressure) {
//     max_pressure = curDepth;
//   }
//   if (curDepth < min_pressure) {
//     min_pressure = curDepth;
//   }
//   if(max_pressure != min_pressure) {
//     pressure = (curDepth - min_pressure)/(max_pressure - min_pressure);
//   }
// }

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
	// if (mode == DEPTH_CTRL_MODE) {
	// 	return desired * mode_max;
	// } else if(mode == POSITION_CTRL_MODE) {
    //   return desired;
    // }

	// return (((desired + 1) * (mode_max - mode_min))/2) + (mode_min);
  return desired;

}

void initController() {
  // btu.init();
  cac.start();
  // btu.setDryMode(false);
}

void runController() {
  // btu.updateAndRunCycle(mode, scaleNumber(desiredNumber));
  cac.set(0.0000009, yaw, freq);
  cac.setVoid();
}

void curateStatusMsg() {
  stat_msg.mode = mode;
  stat_msg.freq = freq;
  stat_msg.yaw = yaw;
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
