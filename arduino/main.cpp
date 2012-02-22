/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

extern "C" {
#include "drivers/pwm.h"
#include "motor.h"
#include "drivers/serial.h"
#include "drivers/power.h"
#include "drivers/servo.h"
#include "comm.h"
#include "main.h"
#include "wheelmon.h"
#include "speedman.h"
#include "drivers/bump.h"
#include "drivers/led.h"
}

#include "twist.h"
#include "sonar.h"
#include "interrupt.h"
#include "gps.h"
#include "steer.h"
#include "imu.h"
#include "publish.h"

#define CLK 16000

// was 119
#define STEER_OFFSET 115

extern volatile int8_t steer;
extern volatile uint32_t ticks;

volatile uint16_t shutdown_count;

volatile int32_t idle_cnt = 0;

extern "C" {
   /* error handler for pure virutal function calls
    *  turn on LED and loop forever */
   void __cxa_pure_virtual() {
      while(1) {
         led_on(); // pure virtual function call
      }
   }
}

char sub_buffer[256];
uint8_t sub_pos = 0;
Packet sub_p(0, 255, sub_buffer);
char sub_type = 0;

// callback on cmd_vel
void vel_cb(Packet & p) {
//   Packet p(buffer, sz);
   // TODO: assert packet size
   target_speed = p.reads16();

   steer = p.reads8();
//   steer = 0;
   servo_set(0, steer + STEER_OFFSET);
}

// subscriber spin loop
void sub_spinOnce() {
   while(rx_ready(BRAIN)) {
      char b = rx_byte(BRAIN);
      if( sub_type == 0 ) {
         sub_type = b;
      } else {
         sub_p.input(b);
         // if we got the end-of-packet flag
         if( b == '\r' ) {
            switch(sub_type) {
               case 'C':
                  vel_cb(sub_p);
                  break;
               default:
                  break;
            }
            sub_p.reset();
            sub_type = 0;
         }
      }
   }
}

// publish idle time data
uint16_t idle;
Publisher<8> idle_pub('I');
uint32_t idle_last = 0;

int main() {
   pwr_on();

   led_init();
   motor_init();

   servo_init();
   DDRC |= (1 << 1);
   servo_map(0, &PORTC, 1);

   servo_set(0, 127);
   battery_init();

   bump_init();

   // serial port 3: bluetooth
   //serial_init(BT);
   // set baud rate: 115.2k baud
   //serial_baud(BT,115200);

   // serial port 0: brain
   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);

   // GPS initialization
   gps_init(GPS);

   // sonar initialization
   sonar_init(SONAR);

   sei(); // enable interrupts

   // imu initialization requires interrupts to be enabled.
   imu_init();

   interrupt_init();

   // power up!
   pwr_on();

   while(1) {
      gps_spinOnce();
      sonar_spinOnce();
      sub_spinOnce();

      _delay_ms(1);
      idle++;
      if( ticks - idle_last > 1000 ) {
         idle_last += 1000;
         idle_pub.reset();
         idle_pub.append(idle);
         idle_pub.finish();
         idle = 0;
      }
   }
   
   // if we're here, we're done. power down.
//   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
