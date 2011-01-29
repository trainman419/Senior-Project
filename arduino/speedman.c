#include "polybot_library/globals.h"
#include "system.h"
#include "speedman.h"
#include "wheelmon.h"

// amount of history to keep for PID integral
#define I_SZ 5

// speed controller mode
u08 mode;

volatile s16 power = 0; 
volatile s16 target_speed;

void speedman() {
   s16 speed = 0;
   s16 oldspeed = 0;
   s16 slip;
   u08 i;
   // keep track of what the speed control thinks we're doing
   //s08 dir = 0; // 0: stopped 1: forward -1: reverse
   mode = M_OFF;

   // true PID control:
   // e: error
   // MV = Kp*e + Ki*integral(e, 0 to t) + Kd*de/dt
   s16 e = 0; // error
   s16 ie = 0; // integral
   s16 de = 0; // derivative
   static s16 Kp = 2; // proportional constant
   static s16 Ki = 0; // integral constant
   static s16 Kd = 0; // derivative constant
   s16 last[I_SZ]; // 1.6 seconds of data; should be enough to compensate for
                  // startup
   u08 last_p = 0;
   for( last_p = 0; last_p < I_SZ; last_p++ ) {
      last[last_p] = 0;
   }
   last_p = 0;

   schedule(100); // 10 times/second

   while(1) {
      led_on();
      //speed = (lspeed + rspeed)/2;
      speed = qspeed;
      //if( dir < 0 ) speed = -speed;
      //if( mode == M_REVERSE  ) speed = -speed;
      slip = abs(lspeed - rspeed);

      e = target_speed - speed; 

      ie = 0;
      for( i=0; i<I_SZ; i++ ) {
         ie += last[i];
      }
      ie /= I_SZ;

      de = e - last[last_p];

      //power += e/Kp + ie/Ki + de/Kd;
      //power += e*Kp + de*Kd;
      power += e*Kp + de*Kd + ie/Ki;

      // hardcoded braking
      /*if( dir == 1 && target_speed == 0 && e < 0 ) {
         power -= 128;
      }*/
      /*if( mode == M_FORWARD && target_speed == 0 && speed != 0 ) {
         power = -1500; // lots of braking
      }*/
      /*if( mode == M_FORWARD && target_speed > (speed + 4) && speed != 0 ) {
         power = -1500; // lots of braking
      }*/
      // hardcoded to disengage brakes
      /*if( mode == M_BRAKE && ( target_speed > 0 || speed == 0 ) ) {
         power = 0;
      }*/
      /*if( mode == M_BRAKE && ( target_speed > speed || speed == 0 ) && power < 0 ) {
         power = 0;
      }*/

      last_p++;
      if( last_p >= I_SZ ) last_p = 0;
      last[last_p] = e;
      

      // hardcoded full stop
      if( target_speed == 0 && speed == 0 ) power = 0;

      // power limits
      if( power/16 > 120 ) power = 16*120;
      if( power/16 < -120 ) power = -16*120;

      /* hardcode to put controller into reverse */
      if( mode == M_BRAKE && target_speed < 0 ) {
         power = 0;
      }

      // track motor controller state
      // motor controller dead range: 113-124
      if( power/16 > 4 ) {
         mode = M_FORWARD;
      } else if( mode == M_FORWARD ) {
         if( power/16 < -6 ) {
            mode = M_BRAKE;
         }
      } else if( mode == M_BRAKE ) {
         if( -6 < power/16 && power/16 < 4 ) {
            mode = M_OFF;
         }
      } else if( mode == M_OFF ) {
         if( power/16 < -6 ) {
            mode = M_REVERSE;
         }
      }

      // output
      set_servo_position(0, power/16+120);
      oldspeed = speed;
      led_off();
      yeild();
   }
}
