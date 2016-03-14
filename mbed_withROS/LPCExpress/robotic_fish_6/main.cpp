//#define COMPILE_BLINK_CODE_ROSSERIAL
#ifdef COMPILE_BLINK_CODE_ROSSERIAL

/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
DigitalOut myled(LED1);

void messageCb( const std_msgs::Empty& toggle_msg) {
    myled = !myled;   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

int main() {
    nh.initNode();
    nh.subscribe(sub);

    while (1) {
        nh.spinOnce();
        wait_ms(1);
    }
}

#endif

//#define acousticControl
//#define serialControl
#define rosControl

#include "mbed.h"
#ifdef acousticControl
#include "AcousticControl/AcousticController.h"
#endif
#ifdef serialControl
#include "SerialControl/SerialController.h"
#endif
#ifdef rosControl
#include "ROSControl/ROSController.h"
#endif

Serial pc(USBTX, USBRX);

int main()
{
	/* ACOUSTIC CONTROL */
	#ifdef acousticControl
	pc.baud(115200);
	// Initialize the acoustic controller
	acousticController.init(&pc); // if no serial object is provided, it will create one on the USB pins
	// Start the controller
	// NOTE this is a blocking method, and if infiniteLoopAcoustic is defined it will run forever (or until low battery callback or button board reset command)
	// It can be stopped by the acousticController.stop() method, but you have to
	//  control threading here to actually be able to call that
	//  The acoustic controller hasn't been tested with multi-threading though
	acousticController.run();
	#endif

	/* SERIAL CONTROL */
	#ifdef serialControl
	pc.baud(115200);
	// Initialize the serial controller
	serialController.init(NULL, &pc);
	// Start the controller
	// NOTE this is a blocking method, and if infiniteLoopSerial is defined it will run forever (or until low battery callback or button board reset command)
	// It can be stopped by the serialController.stop() method, but you have to
	//  control threading here to actually be able to call that
	serialController.run();
	#endif

	/* ROS CONTROL */
	#ifdef rosControl
	pc.baud(115200);
	// Initialize the ROS controller
	rosController.init(NULL, &pc);
	// Start the controller
	// NOTE this is a blocking method, and if infiniteLoopSerial is defined it will run forever (or until low battery callback or button board reset command)
	// It can be stopped by the rosController.stop() method, but you have to
	//  control threading here to actually be able to call that
	rosController.run();
	#endif
}
