#include "mbed.h"
#include <ros.h>
#include <ros/time.h>
#include "AdaBNO055/AdaBNO055.h"
#include "MS5837/MS5837.h"
#include <cmath>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/Vector3Stamped.h>

#define PIN_SDA p28
#define PIN_SCL p27

ros::NodeHandle nh;
PwmOut led_imu(LED1);
PwmOut led_pressure(LED2);
I2CManager manager(PIN_SDA, PIN_SCL);
BNO055 imu(manager);
MS5837 pressureSensor(manager);

sensor_msgs::FluidPressure pressure_msg;
ros::Publisher pressure_pub("pressure", &pressure_msg);

sensor_msgs::Temperature temperature_msg;
ros::Publisher temperature_pub("temperature", &temperature_msg);

geometry_msgs::Vector3Stamped imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

int main() {
  manager.start_comms();
	int imu_status, pressure_status;
  while ((imu_status = imu.begin(OPERATION_MODE_NDOF)) < 0);
  led_imu = 1;
	while ((pressure_status = pressureSensor.init()) < 0);
  led_pressure = 1;
  pressureSensor.start();

  nh.getHardware()->setBaud(115200);
	nh.initNode();
  nh.advertise(pressure_pub);
  nh.advertise(temperature_pub);
  nh.advertise(imu_pub);

	while (1) {
    imu_msg.header.stamp = nh.now();
		Vector euler;
    int res = imu.getVector(VECTOR_EULER, &euler);
    if (res == 0) {
      imu_msg.vector.x = euler[0];
      imu_msg.vector.y = euler[1];
      imu_msg.vector.z = euler[2];
      imu_pub.publish(&imu_msg);
    }
    if (imu_msg.vector.x == 0 || imu_msg.vector.y == 0 || imu_msg.vector.z == 0) {
      led_imu = 0;
    } else {
      led_imu = 1;
    }
		if (pressureSensor.done()) {
      pressure_msg.header.stamp = nh.now();
			pressure_msg.fluid_pressure = pressureSensor.get_pressure();
      pressure_pub.publish(&pressure_msg);
      temperature_msg.header.stamp = nh.now();
      temperature_msg.temperature = pressureSensor.get_temperature();
      temperature_pub.publish(&temperature_msg);
      led_pressure = 1;
		} else {
      led_pressure = 0;
    }

    nh.spinOnce();
    wait_ms(100);
	}
}
