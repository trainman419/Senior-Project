/* speedman.c
 *
 * A speed management thread.
 *
 * Design notes:
 * The intention here is not to provide precise speed control, but rather to
 * provide adequate speed control that is relatively glitch-free and quick to
 * respond to changes in the input, while still being adaptive enough to 
 * respond to and compensate for external forces such as changes in battery 
 * voltage and slopes.
 *
 * Limitations:
 * There is the possibility of overshooting the target speed on startup.
 * This module will not do active braking.
 * It may take some time to stabilize to the desired speed.
 *
 * Strengths:
 * Stopping is immediate
 * When in motion, changes in input will produce immediate, proportional 
 *  changes in the output
 *
 * TODO: test this.
 */
#include "system.h"
#include "speedman.h"
#include "wheelmon.h"
#include "motor.h"

#include <stdio.h>
#include "serial.h"
#include "main.h"

// amount of history to keep for PID integral
#define I_SZ 5

// speed controller mode
uint8_t mode;

volatile int16_t power = 0; 
volatile int16_t target_speed;

uint8_t buf[80];

void speedman() {
   int16_t speed = 0;
   uint8_t i;
   uint16_t mult = 16;
   // keep track of what the speed control thinks we're doing
   //s08 dir = 0; // 0: stopped 1: forward -1: reverse
   mode = M_OFF;

   // true PID control:
   // e: error
   // MV = Kp*e + Ki*integral(e, 0 to t) + Kd*de/dt
   int16_t e = 0; // error

   const static int16_t Kp = 0; // proportional constant
   uint8_t last_p = 0;
   for( last_p = 0; last_p < I_SZ; last_p++ ) {
      last[last_p] = 0;
   }
   last_p = 0;

   schedule(100); // 10 times/second
   //schedule(200); // 5 times/second

   while(1) {
      speed = qspeed;

      e = target_speed - speed; 

      mult += e * Ke;

      if( mult < 1 ) mult = 1;

      power = mult * target_speed;

      // hardcoded full stop
      if( target_speed == 0 && speed == 0 ) power = 0;

      // power limits
      if( power/16 > 50 ) power = 16*50;
      if( power/16 < -50 ) power = -16*50;

      // output
      motor_speed(power/16);

      yeild();
   }
}
