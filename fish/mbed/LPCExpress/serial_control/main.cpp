#include "mbed.h"
#include <ros.h>
#include <ros/time.h>
#include "AdaBNO055/AdaBNO055.h"
#include "MS5837.h"
#include <cmath>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/Vector3Stamped.h>

#define PIN_SDA p28
#define PIN_SCL p27

ros::NodeHandle nh;
PwmOut led_imu(LED1);
PwmOut led_temp(LED2);
PwmOut led_pressure(LED3);
BNO055 imu(PIN_SDA, PIN_SCL);
MS5837 pressureSensor(PIN_SDA, PIN_SCL);

sensor_msgs::FluidPressure pressure_msg;
ros::Publisher pressure_pub("pressure", &pressure_msg);

sensor_msgs::Temperature temperature_msg;
ros::Publisher temperature_pub("temperature", &temperature_msg);

geometry_msgs::Vector3Stamped imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

int main() {
	bool status = imu.begin(OPERATION_MODE_NDOF);
	pressureSensor.MS5837Init();
	pressureSensor.MS5837Start();

	nh.initNode();
  nh.advertise(pressure_pub);
  nh.advertise(temperature_pub);
  nh.advertise(imu_pub);
  unsigned char s, g, a, w;

	while (1) {
		imu.getCalibration(&s, &g, &a, &w);
		if (status){
      imu_msg.header.stamp = nh.now();
			Vector euler = imu.getVector(VECTOR_EULER);
      imu_msg.vector.x = euler[0];
      imu_msg.vector.y = euler[1];
      imu_msg.vector.z = euler[2];

      imu_pub.publish(&imu_msg);
      if (imu_msg.vector.x > 0 || imu_msg.vector.y > 0 || imu_msg.vector.z > 0) {
        led_temp = 1;
      } else {
        led_temp = 0;
      }
      if (euler[0] > 0 || euler[1] > 0 || euler[2] > 0) {
        led_pressure = 1;
      } else {
        led_pressure = 0;
      }
      led_imu = 1;
		} else {
      led_imu = 0;
    }
		if (pressureSensor.MS5837Done()) {
      pressure_msg.header.stamp = nh.now();
			pressure_msg.fluid_pressure = pressureSensor.MS5837_Pressure();
      pressure_pub.publish(&pressure_msg);
      led_pressure = 1;
      temperature_msg.header.stamp = nh.now();
      temperature_msg.temperature = pressureSensor.MS5837_Temperature();
      temperature_pub.publish(&temperature_msg);
      led_temp = 1;
		} else {
      led_temp = 0;
      led_pressure = 0;
    }

    nh.spinOnce();
    wait_ms(1000);
	}
}
