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

int main() {


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

   serial_init_rx(GPS);
   serial_baud(GPS, 4800);
   DDRH |= (1 << 1);
   PORTH &= ~(1 << 1);

   // power up!
   pwr_on();
   
   while(1) {
      if( handle_bluetooth(BRAIN) ) {
         //PORTB |= (1 << 7);
         motor_speed(speed); // this take 30-400 uS
         //PORTB &= ~(1 << 7);
         servo_set(0, 127 + steer);
      }
      /*if( handle_bluetooth(BT) ) {
         PORTB |= (1 << 7);
         motor_speed(speed); // this take 30-400 uS
         PORTB &= ~(1 << 7);
         servo_set(0, 127 + steer);
      }*/
      if( rx_ready(GPS) ) {
         tx_byte(BT, rx_byte(GPS));
      }
   }

   // if we're here, we're done. power down.
   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
