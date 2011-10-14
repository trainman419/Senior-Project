/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <geometry_msgs/Twist.h>

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
#include "gps.h"

#define CLK 16000

#define STEER_OFFSET 119

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

// callback on cmd_vel
void vel_cb( const geometry_msgs::Twist & cmd_vel ) {
   // internal speed specified as 2000/(ms per count)
   // 2 / (sec per count)
   // 2 * counts / sec
   // ( 1 count = 0.03 m )
   // 1/2 * 0.03 m / sec
   // 0.015 m / sec
   // target speed in units of 0.06 m / sec
   target_speed = cmd_vel.linear.x * 66.6667;
   // angular z > 0 is left
   // TODO: derive the algorithm and constraints on steering and implement
   // vr = vl / r
   // r = vl / vr
   steer = -cmd_vel.angular.z;
   float radius;
   if( cmd_vel.angular.z == 0.0 ) {
      steer = 0;
   } else {
      radius = fabs(cmd_vel.linear.x / cmd_vel.angular.z);
      float tmp = pow( radius / 113.36844, -0.8890556);
      if( tmp > 120 ) tmp = 120.0;

      if( cmd_vel.angular.z > 0 ) {
         steer = -tmp;
      } else {
         steer = tmp;
      }
   }
   servo_set(0, steer + STEER_OFFSET);
}
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", & vel_cb);

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

   sei(); // enable interrupts

   nh.initNode();
   nh.advertise(gps_pub);
   nh.advertise(odom_pub);

   nh.subscribe(vel_sub);

   // GPS initialization
   gps_init(GPS);

   interrupt_init();

   // power up!
   pwr_on();

   while(1) {
      gps_spinOnce();
      nh.spinOnce();
   }
   
   // if we're here, we're done. power down.
   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
