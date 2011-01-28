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

// dump the stack
void stack(uint8_t bar) {
   uint8_t * foo = &bar;
   uint8_t tmp = 0;
   uint8_t tmpa = 0;
   static uint8_t * end = (uint8_t*)0x21FF; // top of memory

   char outbuf[7];
   outbuf[0] = '0';
   outbuf[1] = 'x';
   outbuf[4] = '\r';
   outbuf[5] = '\n';
   outbuf[6] = 0;

   tx_byte(BT, '0');
   tx_byte(BT, 'x');
   uint16_t tmp_sp = SP;
   for( tmp=0; tmp < 16; tmp += 4 ) {
      tmpa = (tmp_sp >> (12-tmp));
      tmpa &= 0xF;
      if( tmpa > 0x9 ) {
         tx_byte(BT, tmpa - 0xA + 'A');
      } else {
         tx_byte(BT, tmpa + '0');
      }
   }
   tx_byte(BT, '\r');
   tx_byte(BT, '\n');

   while(foo < end) {
      tmp = *foo;
      tmpa = (tmp & 0xF0) >> 4;
      if( tmpa > 0x9 ) {
         outbuf[2] = tmpa - 0xA + 'A';
      } else {
         outbuf[2] = tmpa + '0';
      }
      tmpa = (tmp & 0x0F);
      if( tmpa > 0x9 ) {
         outbuf[3] = tmpa - 0xA + 'A';
      } else {
         outbuf[3] = tmpa + '0';
      }
      tx_string(BT, outbuf);

      foo++;
   }

   /* stack dump:
      0x21EB      // stack pointer; start of frame (size 8)
0x21EC
0x21ED
0x21EE
0x21EF
0x21F1
0x21F2
0x21F3
0x21F4      0x42        // top of stack; (end of frame)

0x21F5      0xFF        // prologue frame start
0x21F6      0x21
0x21F7      0x06
0x21F8      0x00
0x21F9      0x4E
0x21FA      0x21        // prologue frame end
0x21FB      0x00        ; high
0x21FC      0x01        ; middle
0x21FD      0x75        ; low
0x21FE      0x00
0x21FF      0x00        // bottom of stack
      */
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
                  // FIXME: this will transmit buffered data out of order
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
