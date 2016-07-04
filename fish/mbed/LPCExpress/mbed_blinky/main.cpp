#include "mbed.h"

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);


int main() {

    while(1) {
        myled1 = 1;
        myled2 = 0;
        myled3 = 0;
        myled4 = 0;
        wait(0.2);
        myled1 = 0;
        myled2 = 1;
        myled3 = 0;
        myled4 = 0;
        wait(0.2);
        myled1 = 0;
        myled2 = 0;
        myled3 = 1;
        myled4 = 0;
        wait(0.2);

        myled1 = 0;
        myled2 = 0;
        myled3 = 0;
        myled4 = 1;
        wait(0.2);

    }
}
