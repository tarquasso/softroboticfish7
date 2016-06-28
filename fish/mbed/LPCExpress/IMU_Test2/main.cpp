#include "jtwBNO055/AdaBNO055.h"
#include "mbed.h"
#include <cmath>

#define imuTXPin p28
#define imuRXPin p27

PwmOut calibrationLevel(LED1);
PwmOut pitch(LED2);
PwmOut roll(LED3);
PwmOut yaw(LED4);
BNO055 imu(imuTXPin, imuRXPin);
Serial pc(USBTX, USBRX);
DigitalOut powerIMU(p5);
LocalFileSystem local("local");


Vector quat_to_euler(Quaternion quat) {
    Vector euler;
  euler[0] = atan2(2*(quat.x*quat.y + quat.z*quat.w), 1 - (2*(quat.y*quat.y + quat.z*quat.z)));
  euler[1] = asin(2*(quat.x*quat.z - quat.y*quat.w));
  euler[2] = atan2(2*(quat.x*quat.w + quat.y*quat.z), 1 - (2*(quat.z*quat.z + quat.w*quat.w)));
  return euler;
}


int main() {
  // wait(10);
  powerIMU = 0;
  wait(5);
  powerIMU = 1;
  bool needData = true;
  pc.printf("Starting the BNO055.\r\n");
  bool status = imu.begin(OPERATION_MODE_NDOF, pc);
  pc.printf("SUCCESSFUL? %d\r\n", status);
  while(1) {
    // pc.printf("TEMP: %d\r\n", imu.getTemp());
    Vector euler;
    unsigned char s, g, a, w;
    for(int i = 0; i < 10; i++) {
      euler = imu.getVector(VECTOR_EULER);
      imu.getCalibration(&s, &g, &a, &w);
      calibrationLevel = (float)(s + g + a + w)/12.0;
      pitch = abs(euler[2]/180.0);
      roll = abs(euler[1]/180.0);
      yaw = abs((euler[0]-180.0)/180.0);
      wait_ms(100);

    }
    double tempfloat = 21.2;
    Vector gyro = imu.getVector(VECTOR_GYROSCOPE);
    char calibration[22];
    bool goodData = imu.getSensorOffsets(calibration);
    if(goodData && needData) {
      FILE *fp = fopen("/local/calib", "w");
      for(int i = 0; i < 22; i++) {
        fprintf(fp, "%c", calibration[i]);
      }
      fclose(fp);
    }
    pc.printf("pitch: %.2f, roll: %.2f, yaw: %.2f, test: %.2f\r\n",euler[2], euler[1], euler[0], tempfloat);
    pc.printf("gyro_pitch: %.2f, gyro_roll: %.2f, gyro_yaw: %.2f\r\n", gyro[0], gyro[1], gyro[2]);
    pc.printf("sys: %d, gyro: %d, accel: %d, mag: %d\r\n", s, g, a, w);
    // pc.printf("tempfloat: %.2f\r\n", tempfloat);
  }






  return 0;
}
