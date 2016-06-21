/* Test which brings default HelloWorld project from mbed online compiler
   to be built under GCC.
*/
#include "mbed.h"
#include <ros.h>
#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::Int32 int_msg;
std_msgs::String str_msg;

ros::Publisher chatter("chatter", &int_msg);

PwmOut onesled(LED4);
PwmOut twosled(LED3);
PwmOut foursled(LED2);
PwmOut eightsled(LED1);

int base = 2;
int light_factor = 1;

int counter = 0;
bool saidboom = false;
void messageCb(const std_msgs::Int32& int_msg) {
    counter = int_msg.data;
}

void displayNum(int num) {
    int tmp = num;
    int data[4];
    for(int i = 0; i<4; i++) {
        data[i] = tmp % base;
        tmp /= base;
    }
    onesled = pow((float)data[0] / (base-1), light_factor);
    twosled = pow((float)data[1] / (base-1), light_factor);
    foursled = pow((float)data[2] / (base-1), light_factor);
    eightsled = pow((float)data[3] / (base-1), light_factor);
}

ros::Publisher boom("boomchan", &str_msg);
ros::Subscriber<std_msgs::Int32> sub("numberchan", &messageCb);

char boom_msg[8] = "BOOM!!!";

int main()
{
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(boom);

    while(1) {
        if(counter > 0) {
            counter -= 1;
            saidboom = false;
        } else if (!saidboom) {
            str_msg.data = boom_msg;
            boom.publish( &str_msg );
            saidboom = true;
        }
        displayNum(counter);
        nh.spinOnce();
        wait_ms(1000);
    }
}
