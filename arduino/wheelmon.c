/* wheelmon.c
   The wheel monitoring process and associated globals.

   Author: Austin Hendrix
 */

#include "polybot_library/globals.h"
#include "system.h"

volatile s16 lspeed; /* left wheel speed (Hz) */
volatile s16 rspeed; /* right wheel speed (Hz) */
volatile u16 lcount; /* left wheel count, revolutions */
volatile u16 rcount; /* right wheel count, revolutions */

volatile s16 qspeed; /* quaderature encoder speed */
volatile u16 qcount; /* quaderature encoder 1/4 turn count */

/* extend the OS to run this on a strict schedule: DONE! */
#define WHEELDIV 2000
void wheelmon() {
   u16 lcnt = 0;
   u16 rcnt = 0;
   u08 l, r;
   l = digital(0);
   r = digital(1);

   qcount = 0;

   u08 q1, q2, q1_old, q2_old;
   q1_old = digital(4);
   q2_old = digital(5);
   u16 qcnt = 0;

   while(1) {
      /* read sensors early so we don't get jitter */
      q1 = digital(4);
      q2 = digital(5);
      /* read wheel sensors and update computed wheel speed */
      if( digital(0) == l ) {
         lcnt++;
         if( lspeed > WHEELDIV/lcnt ) lspeed = WHEELDIV/lcnt;
         if( lcnt > WHEELDIV+1) {
            lcnt = WHEELDIV+1;
            lspeed = 0;
         }
      } else {
         if(l) lcount++;
         lspeed = WHEELDIV/lcnt;
         lcnt = 0;
         l = digital(0);
      }

      if( digital(1) == r ) {
         rcnt++;
         if( rspeed > WHEELDIV/rcnt ) rspeed = WHEELDIV/rcnt;
         if( rcnt > WHEELDIV+1 ) {
            rcnt = WHEELDIV+1;
            rspeed = 0;
         }
      } else {
         if(r) rcount++;
         rspeed = WHEELDIV/rcnt;
         rcnt = 0;
         r = digital(1);
      }

      /* read the quaderature encoder on the drive gear to get better
         direction and speed data */
      if( q1 != q1_old ) {
         if( q1 == q2 ) {
            // turning forward
            qspeed = WHEELDIV/qcnt;
         } else {
            // turning backward
            qspeed = -(WHEELDIV/qcnt);
         }
         q1_old = q1;
         qcnt = 0;
         qcount++;
      } else if( q2 != q2_old ) {
         if( q1 == q2 ) {
            // turning backward
            qspeed = -(WHEELDIV/qcnt);
         } else {
            // turning forward
            qspeed = WHEELDIV/qcnt;
         }
         q2_old = q2;
         qcnt = 0;
         qcount++;
      } else {
         qcnt++;
         if( qcnt > WHEELDIV+1 ) {
            qcnt = WHEELDIV+1;
            qspeed = 0;
         } else if( qspeed > 0 ) {
            if( qspeed > WHEELDIV/qcnt ) qspeed = WHEELDIV/qcnt;
         } else {
            if( qspeed < -(WHEELDIV/qcnt) ) qspeed = -(WHEELDIV/qcnt);
         }
      }

      /* updated estimates:
         212 instructions; estimate 2 cycles each
         new estimate 26.5 uS; should still be plenty fast enough
         */
      /* yeild the processor until we need to run again */
      yeild();
   }
}
