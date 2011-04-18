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
#include "bump.h"

#include <stdio.h>
#include "main.h"

#include <math.h>

#define DIV 256

#define abs(a) ((a)>0?(a):-(a))

volatile int16_t power = 0; 
volatile int16_t target_speed;

void speedman() {
   int16_t speed = 0;
   //uint16_t mult = DIV;
   double mult = DIV;

   // true PID control:
   // e: error
   // MV = Kp*e + Ki*integral(e, 0 to t) + Kd*de/dt
   int16_t e = 0; // error

   //const static int16_t Kp = 1; // proportional constant
   const static double Kp = DIV/16; // proportional constant

   schedule(100); // 10 times/second
   //schedule(200); // 5 times/second

   while(1) {
      speed = qspeed;

      e = target_speed - speed; 
      //if( target_speed < 0 ) e = -e;

      //mult += e * Kp;
      if( target_speed != 0 ) {
         mult += Kp * (((double)e) / ((double)target_speed));
         //mult += Kp * e;
      }

      if( mult < 1 ) mult = 1;
      if( fabs(mult*target_speed) > 100.0*DIV ) 
         mult = 100.0*(double)DIV/(double)target_speed;

      power = mult * (double)target_speed;

      // hardcoded full stop
      //if( target_speed == 0 && speed == 0 ) power = 0;

      // power limits
      //if( power/DIV > 100 ) power = DIV*100;
      //if( power/DIV < -100 ) power = -DIV*100;

      if( power > 0 && bump() ) power = 0;

      // output
      motor_speed(power/DIV);

      yeild();
   }
}
