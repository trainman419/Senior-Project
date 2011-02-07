/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "pwm.h"
#include "motor.h"
#include "serial.h"
#include "power.h"
#include "servo.h"
#include "system.h"
#include "comm.h"
#include "main.h"
#include "wheelmon.h"
#include "speedman.h"

#define CLK 16000


int8_t speed;
int8_t steer;

void tx_string(uint8_t port, char * s) {
   int i = 0;
   while(s[i]) {
      tx_byte(port, s[i]);
      i++;
   }
}

volatile uint16_t shutdown_count;

uint8_t buffer[80];

void shutdown(void) {
//   uint8_t i;
   while( shutdown_count == 0 ) {
      // Query command, for debugging
      // read speeds and output them
//      i = sprintf((char*)buffer, "Rspeed: %d, Lspeed: %d, Qspeed: %d\r\n",
//            rspeed, lspeed, qspeed);
//      tx_bytes(BRAIN, buffer, i);

      yeild();
      PORTB |= (1 << 7);
      yeild();
      PORTB &= ~(1 << 7);
   }
   while( shutdown_count > 0 ) {
      //PORTB |= (1 << 7);
      shutdown_count--;
      yeild();
      /*PORTB &= ~(1 << 7);
      shutdown_count--;
      yeild();*/
   }
   // LED ON
   while(1) {
      PORTB |= (1 << 7);
      pwr_off();
      yeild();
   }
}

// too big to stick on the stack
uint8_t laser_buffer[512];

#define MODE_IDLE 0
#define MODE_LASER 1
#define MODE_POWER 2
#define MODE_MOTOR 3
#define MODE_STEER 4
#define MODE_SHUT  5
#define MODE_WAIT 255

int main() {
   DDRB |= 1 << 7;
   motor_init();

   servo_init();
   DDRC |= (1 << 1);
   servo_map(0, &PORTC, 1);

   servo_set(0, 127);
   battery_init();

   system_init();

   // this combination of settings makes this the idle process
   schedule(0); // run all the time
   priority(250); // run only when nothing else wants to

   sei(); // enable interrupts

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

   // initialize the shutdown process
   shutdown_count = 0;
   system(shutdown, 250, 2); // func, schedule, priority

   // GPS initialization
   serial_init_rx(GPS);
   serial_baud(GPS, 4800);
   DDRH |= (1 << 1);
   PORTH &= ~(1 << 1);

   // power up!
   pwr_on();

   system(wheelmon, 1, 1); // wheel monitor; frequent and high priority
   system(speedman, 100, 2); // speed manager; frequency: 10Hz

   //system(brain_rx_thread, 0, 5); // start brain thread
   system(bt_rx_thread, 1, 5);    // start bluetooth thread
   
   // main loop. Manage data flow between bluetooth and computer
   while(1) {
      brain_rx_thread();
   }

   // if we're here, we're done. power down.
   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
