#include "mbed.h"
#include "SerialComm/SerialComm.h"
#include "AdaBNO055/AdaBNO055.h"
#define PIN_SDA p28
#define PIN_SCL p27

PwmOut led_imu(LED1);
PwmOut led_2(LED2);
PwmOut led_3(LED3);
BNO055 imu(PIN_SDA, PIN_SCL);
Serial pc(USBTX, USBRX);

int main() {
  pc.printf("Starting the BNO055.\r\n");
  int status;
  do {
    status = imu.begin(OPERATION_MODE_NDOF);
    pc.printf("SUCCESSFUL? %d\r\n", status);
  } while (status < 0);
  led_imu = 1;

  unsigned char s, g, a, w;
  int res;
  while (1) {
    if (status == 0){
      res = imu.getCalibration(&s, &g, &a, &w);
      if (res < 0) {
        pc.printf("NACK on calibration query\r\n");
      } else {
        pc.printf("calibration: s %d g %d a %d w %d\r\n", s,g,a,w);
      }
      Vector euler;
      res = imu.getVector(VECTOR_EULER, &euler);
      if (res < 0) {
        pc.printf("NACK on vector query\r\n");
      } else {
        pc.printf("pitch: %.2f, roll: %.2f, yaw: %.2f\r\n",euler[2], euler[1], euler[0]);
      }

      if (euler[0] == 0 || euler[1] == 0 || euler[2] == 0) {
        led_2 = 0;
      } else {
        led_2 = 1;
      }
    } else {
      led_imu = 0;
    }

    wait_ms(1000);
  }
}
