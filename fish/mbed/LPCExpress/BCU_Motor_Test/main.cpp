// test file for BuoyancyControlUnit class functions
// NOTE look at the below h files to define whether that control mode is enabled

#include "mbed.h"
#include "QEI/QEI.h"

#define bcuPwmPin p22
#define bcuDirAPin p11
#define bcuDirBPin p12
#define bcuCurrentPin p20

#define valvePwmPin p21

#define encoderPinA p16
#define encoderPinB p17
#define count2rev 12 // https://www.pololu.com/product/3081/blog
#define gearRatio 75 // need to check

bool bcuDir = 0;

Serial pc(USBTX, USBRX);
Ticker run_timer;

Timer timer;
PwmOut bcuPWM(bcuPwmPin);
QEI bcuEncoder(encoderPinA, encoderPinB, NC, count2rev, QEI::X4_ENCODING);
DigitalOut bcuDirA(bcuDirAPin);
DigitalOut bcuDirB(bcuDirBPin);
AnalogIn bcuCurrent(bcuCurrentPin);

PwmOut valvePWM(valvePwmPin);


float Vin;

void run_func() {
	float t = timer.read();
	float cur = bcuCurrent.read();
	int pos = bcuEncoder.getPulses();
	if(pos >= 0){
		bcuPWM.write(Vin);
		pc.printf("%f\t %f\t %d\t %f\t %d\n", t, Vin, bcuDir, cur, pos);
	} else {
		pc.printf("returned to origin, pos %d\n", pos);
		bcuPWM.write(0.0);
		run_timer.detach();
	}

}

int main() {
	pc.baud(19200);
	bcuEncoder.reset();

	valvePWM = 0;
	bcuPWM = Vin;

	bcuDirA = bcuDir; // set motor direction
	bcuDirB = !bcuDir;

	float input;
	while (1) {
		pc.scanf("%f", &input); // use pressure sensor
		if(input > 1){
			bcuDir = !bcuDir; // reverse the motor, same voltage
			bcuDirA = bcuDir;
			bcuDirB = !bcuDir;
		} else if(input == 0){
			pc.printf("motor stopped");
			bcuPWM = 0.0; // stop the motor
			run_timer.detach();
		} else{
			Vin = input;
			run_timer.attach(&run_func, .05);
			timer.start();
		}
	}
}
