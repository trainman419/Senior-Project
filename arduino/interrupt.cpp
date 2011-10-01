/*
 * New interrupt-based wheel and speed management; replaces the RTOS
 */

#include <avr/interrupt.h>

//#include "protocol.h"

extern "C" {
#include "serial.h"
#include "motor.h"
#include "bump.h"
}

#include "ros.h"
#include <dagny_msgs/Odometry.h>

uint32_t ticks = 0;

uint8_t input, input_old;

#define L 0x80
#define R 0x40
// front
#define Q1 0x20
// back
#define Q2 0x10

#define WHEELDIV 2000

// wheel counter variables
uint16_t lcnt = 0;
uint16_t rcnt = 0;
int16_t qcnt = 0;

uint8_t q1;
uint8_t q2;

volatile uint16_t lspeed; /* left wheel speed (Hz) */
volatile uint16_t rspeed; /* right wheel speed (Hz) */
volatile uint16_t lcount; /* left wheel count, revolutions */
volatile uint16_t rcount; /* right wheel count, revolutions */

volatile int16_t qspeed; /* quaderature encoder speed */
volatile int16_t qcount; /* quaderature encoder 1/4 turn count */


#define DIV 256

//#define abs(a) ((a)>0?(a):-(a))

// speed management variables
volatile int16_t power = 0; 
volatile int16_t target_speed;

int16_t speed = 0;
int16_t mult = DIV;
int16_t e = 0; // error

// odometry transmission variables
volatile uint16_t odom_sz = 0;
volatile int8_t steer;
unsigned char out_buffer[128]; // output buffer
dagny_msgs::Odometry odom;
ros::Publisher odom_pub("odometry", &odom);

/* set up interrupt handling */
void interrupt_init(void) {
   // set motor pins as input
   DDRC &= ~( L | R | Q1 | Q2);
   // set pull-ups on inputs
   PORTC |= ( L | R | Q1 | Q2);

   input = input_old = PINC;

   // set up timer interrupts
   // fast PWM mode; interrupt and reset when counter equals OCR0A
   // prescalar 64
   TCCR0A = (1 << WGM01 | 1 << WGM00);
   TCCR0B = (1 << WGM02 | 1 << CS01 | 1 << CS00);
   // interrupt on "overflow" (counter match)
   TIMSK0 = (1 << TOIE0);
   OCR0A  = 249; // 250 counts per tick
}

/* interrupt routine */
/* TIMER0 OVF */
ISR(TIMER0_OVF_vect) {
   ticks++;
   // every time; read wheel sensors and compute counts and speed.
   {
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
            qcount++;
         } else {
            // turning backward
            qspeed = -(WHEELDIV/qcnt);
            qcount--;
         }
         qcnt = 0;
      } else if( (input ^ input_old) & Q2 ) { // Q2 change
         if( q1 == q2 ) { // q1 == q2
            // turning backward
            qspeed = -(WHEELDIV/qcnt);
            qcount--;
         } else {
            // turning forward
            qspeed = WHEELDIV/qcnt;
            qcount++;
         }
         qcnt = 0;
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
   }
   // end wheel encoder processing

   // speed management; run at 10Hz
   const static int16_t Kp = DIV/16; // proportional constant
   if( ticks % 100 == 0 ) {
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
   }

   // wheel encoder and speed transmit; 20Hz
   if( ticks % 50 == 0 ) {
      // TODO: do this with rosserial instead
      /*
      odom.reset();
      odom.append((uint16_t)rcount);
      odom.append((uint16_t)lcount);
      odom.append((uint16_t)qcount);
      odom.append((int16_t)rspeed);
      odom.append((int16_t)lspeed);
      odom.append((int16_t)qspeed);
      odom.append(steer);
      odom.finish();
      odom_sz = odom.outsz();
      tx_buffer(BRAIN, (uint8_t *)odom.outbuf(), odom_sz);
      */
      odom.lspeed = lspeed;
      odom.rspeed = rspeed;
      odom.qspeed = qspeed;
      odom.lcount = lcount;
      odom.rcount = rcount;
      odom.qcount = qcount;
      odom.steer = steer;
      //odom_sz += odom.serialize(out_buffer);
      odom_pub.publish(&odom);
   }
}
