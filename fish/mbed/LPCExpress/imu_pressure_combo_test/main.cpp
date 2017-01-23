#include "mbed.h"
#include "SerialComm/SerialComm.h"
#include "AdaBNO055/AdaBNO055.h"
#include "MS5837/MS5837.h"
#define PIN_SDA p28
#define PIN_SCL p27

PwmOut led_imu(LED1);
PwmOut led_pressure(LED3);
PwmOut led_2(LED2);
PwmOut led_4(LED4);
Serial pc(USBTX, USBRX);
I2CManager manager(PIN_SDA, PIN_SCL, &pc);
BNO055 imu(manager);
MS5837 pressureSensor(manager);

int main() {
 //  pc.printf("reset\r\n");
	// imu.reset();
 //  pc.printf("checking\r\n");
 //  if (imu.check()) {
 //    pc.printf("imu passes check\r\n");
 //    led_imu = 1;
 //  } else {
 //    pc.printf("imu fails check\r\n");
 //    led_imu = 0;
 //  }

	// while (1) {
 //    imu.setmode(OPERATION_MODE_NDOF);
 //    imu.get_calib();
 //    imu.get_angles();
 //    pc.printf("%0z %5.1d %5.1d %5.1d\r\n",imu.calib,imu.euler.roll,imu.euler.pitch,imu.euler.yaw);
 //    if (imu.euler.roll == 0 || imu.euler.pitch == 0 || imu.euler.yaw == 0) {
 //      led_2 = 0;
 //    } else {
 //      led_2 = 1;
 //    }

 //    wait_ms(1000);
	// }

  manager.start_comms();

  pc.printf("Starting the BNO055.\r\n");
  int imu_status;
  while ((imu_status = imu.begin(OPERATION_MODE_NDOF)) < 0) {
    pc.printf("SUCCESSFUL? %d\r\n", imu_status);
  }
  if (imu_status == 0) {
    led_imu = 1;
  } else {
    led_imu = 0;
  }
  pc.printf("Starting the MS5837.\r\n");
  int pressure_status = pressureSensor.init();
  pc.printf("SUCCESSFUL? %d\r\n", pressure_status);
  if (pressure_status == 0) {
    led_pressure = 1;
  } else {
    led_pressure = 0;
  }
  pressureSensor.start();

  unsigned char s, g, a, w;
  bool success;
  while (1) {
    if (imu_status == 0){
      success = imu.getCalibration(&s, &g, &a, &w);
      if (success == 0) {
        pc.printf("calibration: s %d g %d a %d w %d\r\n", s,g,a,w);
      } else {
        pc.printf("NACK on calibration query\r\n");
      }
      Vector euler;
      success = imu.getVector(VECTOR_EULER, &euler);
      if (success < 0) {
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
    if (pressure_status == 0) {
      success = pressureSensor.done();
      if (success) {
        float pressure = pressureSensor.get_pressure();
        float temperature = pressureSensor.get_temperature();
        pc.printf("pressure: %f, temp: %f\r\n", pressure, temperature);
        led_4 = 1;
      } else {
        pc.printf("pressure/temp query not ready\r\n");
        led_4 = 0;
      }
    }

    wait_ms(100);
  }
}
