/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#include <avr/io.h>
#include <avr/interrupt.h>
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

/*void tx_batteries(uint8_t port) {
   uint8_t mainb, motor;
   mainb = main_battery();
   motor = motor_battery();
   // output format:
   // Main  XXX
   // Motor XXX
   // 11 characters per line
   //
   // I'm kinda assuming that format[] will be an immutable buffer in flash
   char format[] = "Main  XXX\r\nMotor XXX\r\n";
   char output[24]; // a few spares, just in case
   uint8_t i;
   for( i=0; format[i]; i++ )
      output[i] = format[i];
   output[i] = 0;

   output[6] = (mainb/100) + '0';
   output[7] = ((mainb/10) % 10) + '0';
   output[8] = (mainb % 10) + '0';

   output[17] = (motor/100) + '0';
   output[18] = ((motor/10) % 10) + '0';
   output[19] = (motor % 10) + '0';

   tx_string(port, output);
}*/

/* read serial port, parse data, send results */
uint8_t handle_bluetooth(uint8_t port) {
   uint8_t res = 0;
   if( rx_ready(port) ) {
      uint8_t bt = rx_byte(port);
      switch(bt) {
         case 'a':
            res = 1;
            steer -= 10;
            tx_string(port, "left\r\n");
            break;
         case 'd':
            res = 1;
            steer += 10;
            tx_string(port, "right\r\n");
            break;
         case 'w':
            res = 1;
            speed += 5;
            tx_string(port, "faster\r\n");
            break;
         case 's':
            res = 1;
            speed -= 5;
            tx_string(port, "slower\r\n");
            break;
         case ' ':
            res = 1;
            speed = 0;
            steer = 0;
            tx_string(port, "stop\r\n");
            break;
      }
      if( speed > 50 ) speed = 50;
      if( speed < -50 ) speed = -50;
      if( steer > 100 ) steer = 100;
      if( steer < -100 ) steer = -100;

      int8_t tmp = speed;
      if(speed < 0) {
         tx_byte(port, '-');
         tmp = -speed;
      }
      tx_byte(port, '0' + (tmp/100));
      tx_byte(port, '0' + ((tmp/10)%10));
      tx_byte(port, '0' + (tmp%10));
      tx_byte(port, '\r');
      tx_byte(port, '\n');

      //tx_batteries(port);
   }
   return res;
}

// too big to stick on the stack
uint8_t laser_buffer[256];

#define MODE_IDLE 0
#define MODE_LASER 1
#define MODE_POWER 2
#define MODE_WAIT 255

int main() {
   uint16_t laser_index;
   uint16_t laser_count;
   
   uint8_t brain_rx_mode;
   uint8_t brain_tx_mode;

   uint8_t bt_rx_mode;
   uint8_t bt_tx_mode;

   DDRB |= 1 << 7;
   motor_init();

   servo_init();
   DDRC |= (1 << 1);
   //servo_map(0, &PORTC, 1);

   servo_set(0, 127);
   battery_init();

   system_init();

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

   // GPS initialization
   /*serial_init_rx(GPS);
   serial_baud(GPS, 4800);
   DDRH |= (1 << 1);
   PORTH &= ~(1 << 1);*/

   // power up!
   pwr_on();

   laser_index = 0;
   laser_count = 0;
   brain_rx_mode = MODE_IDLE;
   brain_tx_mode = MODE_IDLE;
   bt_rx_mode = MODE_IDLE;
   bt_tx_mode = MODE_IDLE;
   //uint8_t input;
   
   // main loop. Manage data flow between bluetooth and computer
   while(1) {

      /*input = rx_byte(BRAIN);
      tx_byte(BT,input);*/

      // we're losing data no matter how we transfer it because the serial
      //  driver is too slow; it can't handle 115200 in and out for 
      //  extended periods

      tx_byte(BT, 'L');
      for( laser_index=0; laser_index < 256; laser_index++) {
         tx_byte(BT, 64);
      }
      tx_byte(BT, '\r');

      // receive data from the brain and process it
      /*if( rx_ready(BRAIN) ) {
         input = rx_byte(BRAIN);
         // at this point, we expect two types of messages: laser and shutdown
         switch(brain_rx_mode) {
            case MODE_IDLE:
               switch(input) {
                  case 'L':
                     brain_rx_mode = MODE_LASER;
                     laser_index = 0;
                     laser_count = 0;
                     break;
                  case 'P':
                     brain_rx_mode = MODE_POWER;
                     tx_string(BRAIN, "power\n\r");
                     break;
               }
               break;
            case MODE_LASER:
               laser_buffer[laser_index++] = input;
               laser_count++;
               if( bt_tx_mode == MODE_IDLE ) {
                  bt_tx_mode = MODE_LASER;
                  tx_byte(BT, 'L');
               }
               if( bt_tx_mode == MODE_LASER ) {
                  // FIXME: this will transmit buffered data out of order
                  while( laser_index > 0 && tx_ready(BT) ) {
                     laser_index--;
                     tx_byte(BT, laser_buffer[laser_index]);
                  }
               }
               if( laser_count == 256 && bt_tx_mode == MODE_LASER ) {
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
                  tx_string(BRAIN, "ready\n\r");
               }
               break;
         }
      }*/

      // TODO: receive data from BT and process/pass it
   }

   // if we're here, we're done. power down.
   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
