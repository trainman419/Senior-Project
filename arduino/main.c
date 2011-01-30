/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pwm.h"
#include "motor.h"
#include "serial.h"
#include "power.h"
#include "servo.h"
#include "system.h"

#define CLK 16000

#define BRAIN 0
#define BT 3
#define GPS 2
#define SONAR 1

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

void shutdown(void) {
   while( shutdown_count == 0 ) {
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
   uint16_t laser_rx_index;
   uint16_t laser_tx_index;
   uint16_t laser_count;
   
   uint8_t brain_rx_mode;
   uint8_t brain_tx_mode;

   uint8_t bt_rx_mode;
   uint8_t bt_tx_mode;

   uint8_t z_count = 0;

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
   // LED pwm setup
   /*pwm_init(PWM13);
   pwm_set_freq(1, 200);
   pwm_set_duty(PWM13, 0.5);*/

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

   laser_rx_index = 0;
   laser_tx_index = 0;
   laser_count = 0;
   brain_rx_mode = MODE_IDLE;
   brain_tx_mode = MODE_IDLE;
   bt_rx_mode = MODE_IDLE;
   bt_tx_mode = MODE_IDLE;
   uint8_t input;
   
   // main loop. Manage data flow between bluetooth and computer
   while(1) {

      // receive data from the brain and process it
      if( rx_ready(BRAIN) ) {
         input = rx_byte(BRAIN);
         // at this point, we expect two types of messages: laser and shutdown
         switch(brain_rx_mode) {
            case MODE_IDLE:
               switch(input) {
                  case 'L':
                     brain_rx_mode = MODE_LASER;
                     laser_rx_index = 0;
                     laser_tx_index = 0;
                     laser_count = 0;
                     break;
                  case 'P':
                     brain_rx_mode = MODE_POWER;
                     tx_string(BRAIN, "power\n\r");
                     break;
               }
               break;
            case MODE_LASER:
               laser_buffer[laser_rx_index++] = input;
               laser_count++;
               if( bt_tx_mode == MODE_IDLE ) {
                  bt_tx_mode = MODE_LASER;
                  tx_byte(BT, 'L');
               }
               if( bt_tx_mode == MODE_LASER ) {
                  while( laser_tx_index < laser_rx_index && tx_ready(BT) ) {
                     tx_byte(BT, laser_buffer[laser_tx_index]);
                     laser_tx_index++;
                  }
               }
               if( laser_count == 512 && bt_tx_mode == MODE_LASER ) {
                  tx_byte(BT, '\r');
                  bt_tx_mode = MODE_IDLE;
                  brain_rx_mode = MODE_WAIT;
               }
               break;
            case MODE_POWER:
               switch(input) {
                  case '0':
                     break;
                  case '1':
                     pwr_on();
                     tx_string(BRAIN, "power on\n\r");
                     break;
                  case '2':
                     pwr_sleep();
                     tx_string(BRAIN, "power sleep\n\r");
                     break;
                  case '3':
                     // TODO: work out shutdown timer and set it here
                     break;
               }
               brain_rx_mode = MODE_WAIT;
               break;
            case MODE_WAIT:
               if( input == '\n' || input == '\r' ) {
                  brain_rx_mode = MODE_IDLE;
                  //tx_string(BRAIN, "ready\n\r");
               }
               break;
         }
      }

      // receive data from BT and process/pass it
      if( rx_ready(BT) ) {
         input = rx_byte(BT);
         // at this point, we expect several messages:
         // * shutdown
         // * motor control
         // * steering control
         switch(bt_rx_mode) {
            case MODE_IDLE:
               switch(input) {
                  case 'Z':
                     // send shutdown to computer
                     bt_rx_mode = MODE_SHUT;
                     z_count = 1;
                     break;
                  case 'M':
                     bt_rx_mode = MODE_MOTOR;
                     break;
                  case 'S':
                     bt_rx_mode = MODE_STEER;
                     break;
               }
               break;
            case MODE_MOTOR:
               {
               int8_t speed = (int8_t)input;
               speed = speed>50?50:speed;
               speed = speed<-50?-50:speed;
               motor_speed(speed);

               bt_rx_mode = MODE_WAIT;
               }
               break;
            case MODE_STEER:
               servo_set(0, input);
               bt_rx_mode = MODE_WAIT;
               break;
            case MODE_SHUT:
               if( input == 'Z' ) {
                  z_count++;
               } else if( input == '\r' && z_count == 9 ) {
                  tx_string(BRAIN, "ZZZZZZZZ\r");
                  // launch the shutdown timer
                  shutdown_count = 4*60; // 60-second timer 
                  bt_rx_mode = MODE_IDLE;
                  z_count = 0;
               } else {
                  bt_rx_mode = MODE_WAIT;
                  z_count = 0;
               }
               break;
            case MODE_WAIT:
               if( input == '\n' || input == '\r' ) {
                  bt_rx_mode = MODE_IDLE;
               }
               break;
         }
      }
   }

   // if we're here, we're done. power down.
   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
