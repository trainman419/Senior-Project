/* wheelmon.c
   The wheel monitoring process and associated globals.

   Author: Austin Hendrix
 */

#include <avr/io.h>
#include "system.h"

volatile int16_t lspeed; /* left wheel speed (Hz) */
volatile int16_t rspeed; /* right wheel speed (Hz) */
volatile uint16_t lcount; /* left wheel count, revolutions */
volatile uint16_t rcount; /* right wheel count, revolutions */

volatile int16_t qspeed; /* quaderature encoder speed */
volatile uint16_t qcount; /* quaderature encoder 1/4 turn count */

// TODO: check the wiring diagram and make sure these are right.
#define L 0x80
#define R 0x40
// front
#define Q1 0x20
// back
#define Q2 0x10

/* extend the OS to run this on a strict schedule: DONE! */
#define WHEELDIV 2000
void wheelmon() {
   uint16_t lcnt = 0;
   uint16_t rcnt = 0;
   uint16_t qcnt = 0;

   uint8_t input, input_old;

   uint8_t q1;
   uint8_t q2;

   // set motor pins as input
   DDRC &= ~( L | R | Q1 | Q2);
   // set pull-ups on inputs
   PORTC |= ( L | R | Q1 | Q2);

   input = input_old = PINC;

   while(1) {
      /* read sensors early so we don't get jitter */
      input = PINC;
      /* read wheel sensors and update computed wheel speed */
      if( ~(input ^ input_old) & L ) {
         lcnt++;
         if( lspeed > WHEELDIV/lcnt ) lspeed = WHEELDIV/lcnt;
         if( lcnt > WHEELDIV+1) {
            lcnt = WHEELDIV+1;
            lspeed = 0;
         }
      } else {
         if(input & L) lcount++;
         lspeed = WHEELDIV/lcnt;
         lcnt = 0;
      }

      if( ~(input ^ input_old) & R ) {
         rcnt++;
         if( rspeed > WHEELDIV/rcnt ) rspeed = WHEELDIV/rcnt;
         if( rcnt > WHEELDIV+1 ) {
            rcnt = WHEELDIV+1;
            rspeed = 0;
         }
      } else {
         if(input & R) rcount++;
         rspeed = WHEELDIV/rcnt;
         rcnt = 0;
      }

      q1 = (input >> 5) & 0x1;
      q2 = (input >> 4) & 0x1;
      /* read the quaderature encoder on the drive gear to get better
         direction and speed data */
      if( (input ^ input_old) & Q1 ) { // Q1 change
         if( q1 == q2 ) { // q1 == q2
            // turning forward
            qspeed = WHEELDIV/qcnt;
         } else {
            // turning backward
            qspeed = -(WHEELDIV/qcnt);
         }
         qcnt = 0;
         qcount++;
      } else if( (input ^ input_old) & Q2 ) { // Q2 change
         if( q1 == q2 ) { // q1 == q2
            // turning backward
            qspeed = -(WHEELDIV/qcnt);
         } else {
            // turning forward
            qspeed = WHEELDIV/qcnt;
         }
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
      input_old = input;

      /* updated estimates:
         212 instructions; estimate 2 cycles each
         new estimate 26.5 uS; should still be plenty fast enough
         */
      /* yeild the processor until we need to run again */
      yeild();
   }
}
