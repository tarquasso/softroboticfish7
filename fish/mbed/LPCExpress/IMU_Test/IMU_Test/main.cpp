#include "BNO055.h"
#include "mbed.h"
#include <cmath>

#define imuTXPin p28
#define imuRXPin p27

PwmOut calibrationLevel(LED1);
PwmOut pitch(LED2);
PwmOut roll(LED3);
PwmOut yaw(LED4);
BNO055 imu(imuTXPin, imuRXPin);

angles quat_to_euler(quaternion quat) {
  angles euler = angles();
  euler.roll = atan2(2*(quat.x*quat.y + quat.z*quat.w), 1 - (2*(quat.y*quat.y + quat.z*quat.z)));
  euler.pitch = asin(2*(quat.x*quat.z - quat.y*quat.w));
  euler.yaw = atan2(2*(quat.x*quat.w + quat.y*quat.z), 1 - (2*(quat.z*quat.z + quat.w*quat.w)));
  return euler;
}


int main() {
  imu.reset();

  while(1) {
    imu.get_calib();
    imu.get_quat();
    angles euler = quat_to_euler(imu.quat);
    roll = abs(euler.roll / (M_PI));
    pitch = abs(euler.pitch / (M_PI));
    yaw = abs(euler.yaw / (M_PI));
    calibrationLevel = (float) (imu.calib) / 4.0;
    wait(1);
  }

  return 0;
}
