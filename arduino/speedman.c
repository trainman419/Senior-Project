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

// locals in global scope. easier to keep track of.
   int16_t speed = 0;
   int16_t mult = DIV;

   // true PID control:
   // e: error
   // MV = Kp*e + Ki*integral(e, 0 to t) + Kd*de/dt
   int16_t e = 0; // error

   const static int16_t Kp = DIV/16; // proportional constant

void speedman() {
   schedule(100); // 10 times/second
   //schedule(200); // 5 times/second

   while(1) {
      // reflex: stop if we bump into something
      if( target_speed > 0 && bump() ) {
         power = 0;
      } else {
         speed = qspeed;

         e = target_speed - speed; 

         if( target_speed != 0 ) {
            e = Kp * e / target_speed;
            if( e > DIV ) e = DIV;
            if( e < -DIV ) e = -DIV;
            mult += e;
         }

         if( mult < 1 ) mult = 1;
         if( abs(mult*target_speed) > 100*DIV ) 
            mult = 100*DIV/target_speed;

         power = mult * (double)target_speed;
      }

      // output
      motor_speed(power/DIV);

      yeild();
   }
}
