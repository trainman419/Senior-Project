/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>
#include <stdio.h>

#include "ros.h"

extern "C" {
#include "pwm.h"
#include "motor.h"
#include "serial.h"
#include "power.h"
#include "servo.h"
#include "comm.h"
#include "main.h"
#include "wheelmon.h"
#include "speedman.h"
#include "compass.h"
#include "bump.h"
}

#include "interrupt.h"
#include "protocol.h"
#include "gps.h"

#define CLK 16000

extern volatile int8_t steer;
extern volatile uint32_t ticks;

volatile uint16_t shutdown_count;

volatile int32_t idle_cnt = 0;

extern "C" {
   /* error handler for pure virutal function calls
    *  turn on LED and loop forever */
   void __cxa_pure_virtual() {
      while(1) {
         DDRB |= (1 << PB7);
         PORTB |= (1 << PB7);
      }
   }
}

ros::NodeHandle nh;
Packet<16> c_pack('C');
Packet<16> battery('b');

inline void writes16(int16_t s, uint8_t * buf) {
   buf[0] = s & 0xFF;
   buf[1] = (s >> 8) & 0xFF;
}

int main() {
   DDRB |= 1 << 7;
   motor_init();

   servo_init();
   DDRC |= (1 << 1);
   servo_map(0, &PORTC, 1);

   servo_set(0, 127);
   battery_init();

   compass_init();

   bump_init();

   interrupt_init();

   // serial port 3: bluetooth
   serial_init(BT);
   // set baud rate: 115.2k baud
   serial_baud(BT,115200);

   // serial port 0: brain
   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);
   // do some serial init manually
   //  double-speed for more accurate baud rate
   UCSR0A |= (1 << U2X0);
   UBRR0 = 16;

   nh.initNode();
   nh.advertise(odom_pub);

   sei(); // enable interrupts

   // GPS initialization
   gps_init(GPS);

   // power up!
   pwr_on();

   while(1) {
      nh.spinOnce();
   }
   
   // if we're here, we're done. power down.
   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
