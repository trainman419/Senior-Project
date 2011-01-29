#include "system.h"
#include "speedman.h"
#include "wheelmon.h"

// amount of history to keep for PID integral
#define I_SZ 5

// speed controller mode
uint8_t mode;

volatile int16_t power = 0; 
volatile int16_t target_speed;

void speedman() {
   int16_t speed = 0;
   int16_t oldspeed = 0;
   uint8_t i;
   // keep track of what the speed control thinks we're doing
   //s08 dir = 0; // 0: stopped 1: forward -1: reverse
   mode = M_OFF;

   // true PID control:
   // e: error
   // MV = Kp*e + Ki*integral(e, 0 to t) + Kd*de/dt
   int16_t e = 0; // error
   int16_t ie = 0; // integral
   int16_t de = 0; // derivative

   // TODO: tune the gains for the new motor controller
   static int16_t Kp = 2; // proportional constant
   static int16_t Ki = 0; // integral constant
   static int16_t Kd = 0; // derivative constant
   int16_t last[I_SZ]; // 1.6 seconds of data; should be enough to compensate for
                  // startup
   uint8_t last_p = 0;
   for( last_p = 0; last_p < I_SZ; last_p++ ) {
      last[last_p] = 0;
   }
   last_p = 0;

   schedule(100); // 10 times/second

   while(1) {
      speed = qspeed;

      e = target_speed - speed; 

      ie = 0;
      for( i=0; i<I_SZ; i++ ) {
         ie += last[i];
      }
      ie /= I_SZ;

      de = e - last[last_p];

      power += e*Kp + de*Kd + ie/Ki;


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
      yeild();
   }
}
