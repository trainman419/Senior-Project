/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <geometry_msgs/Twist.h>

#include <stdlib.h>
#include <util/delay.h>


#include "ros.h"
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>

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

#include "sonar.h"
#include "interrupt.h"
#include "gps.h"
#include "steer.h"

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
         led_on();
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
   // 1/2 * 0.032 m / sec
   // 0.016 m / sec
   // target speed in units of 0.016 m / sec
   target_speed = cmd_vel.linear.x * 62.5;
   // angular z > 0 is left
   // TODO: derive the algorithm and constraints on steering and implement
   // vr = vl / r
   // r = vl / vr
   if( cmd_vel.angular.z == 0.0 ) {
      steer = 0;
   } else {
      float radius = fabs(cmd_vel.linear.x / cmd_vel.angular.z);
      int16_t tmp = radius2steer(radius);

      if( tmp < -STEER_OFFSET ) 
         tmp = -STEER_OFFSET;
      if( tmp > (255 - STEER_OFFSET) ) 
         tmp = 255 - STEER_OFFSET;

      if( cmd_vel.angular.z > 0 ) {
         steer = -tmp;
      } else {
         steer = tmp;
      }
   }
   servo_set(0, steer + STEER_OFFSET);
}
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", & vel_cb);

void steer_cb( const std_msgs::Int8 & s ) {
   steer = s.data;
   servo_set(0, steer + STEER_OFFSET);
}
ros::Subscriber<std_msgs::Int8> steer_sub("steer", &steer_cb);

// publish battery state
std_msgs::Int8 battery;
ros::Publisher battery_pub("battery", &battery);

// publish idle time data
std_msgs::UInt16 idle;
ros::Publisher idle_pub("avr_idle", &idle);
uint32_t idle_last = 0;


// statically-allocate space for malloc to work from
//char buffer[BUFSZ];

int main() {
   // nothing to see here. move along.
   // setting up malloc to only use our internal buffer
   //__malloc_heap_start = buffer;
   //__malloc_heap_end = buffer + BUFSZ;
   //for( int i=0; i < BUFSZ; ++i ) {
   //  buffer[i] = 0;
   //}

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

   nh.initNode();
   nh.advertise(gps_pub);
   nh.advertise(odom_pub);
//   nh.advertise(battery_pub);
   nh.advertise(idle_pub);
   nh.advertise(sonar_pub);

   nh.subscribe(vel_sub);
   nh.subscribe(steer_sub);

   interrupt_init();

   // power up!
   pwr_on();

   while(1) {
      gps_spinOnce();
      sonar_spinOnce();
      nh.spinOnce();

      _delay_ms(1);
      idle.data++;
      if( ticks - idle_last > 1000 ) {
         idle_last += 1000;
         idle_pub.publish(&idle);
         idle.data = 0;
      }
      // currently about 740 idle ticks
   }
   
   // if we're here, we're done. power down.
   pwr_off();
   // loop forever, in case the arduino is on external power
   while(1);
}
