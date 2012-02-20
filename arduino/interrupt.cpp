/*
 * New interrupt-based wheel and speed management; replaces the RTOS
 */

#include <avr/interrupt.h>
#include <math.h>

extern "C" {
#include "drivers/serial.h"
#include "motor.h"
#include "drivers/bump.h"
}

#include "steer.h"
#include "imu.h"
#include "publish.h"

#define abs(x) ((x)>0?(x):-(x))

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

float x, y, yaw; /* position */
int16_t old_qcount; /* for updating odometry output */

#define DIV 256

// speed management variables
volatile int16_t power = 0; 
volatile int16_t target_speed;

int16_t speed = 0;
int16_t mult = DIV/2;
int16_t e = 0; // error

// odometry transmission variables
volatile uint16_t odom_sz = 0;
volatile int8_t steer;
Packet<32> odom('O');

// 0.03 meters per tick
#define Q_SCALE 0.032

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

      // enable nested interrupts now that we've read our input
      sei();

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
         } else {
            mult = DIV/2;
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
      double r = steer2radius(steer);

      float speed = qspeed * (Q_SCALE * 0.5);
      odom.reset();
      odom.append(speed); // linear speed
      if( steer == 0 ) {
         odom.append(0.0f);
      } else {
         odom.append(speed / r); // angular speed
      }

      // if we've moved, update position
      if( old_qcount != qcount ) {
         float d = (qcount - old_qcount) * Q_SCALE;
         float dx, dy, dt;
         if( steer == 0 ) {
            dx = d * cos(yaw);
            dy = d * sin(yaw);
            dt = 0.0;
         } else {
            dt = d / r;
            float theta_c1;
            float theta_c2;
            if( steer > 0 ) {
               // turning right
               theta_c1 = yaw + M_PI/2;
            } else {
               // turning left
               dt = -dt;
               theta_c1 = yaw - M_PI/2;
            }
            theta_c2 = theta_c1 - dt;

            dx = r * (cos(theta_c2) - cos(theta_c1));
            dy = r * (sin(theta_c2) - sin(theta_c1));
         }

         x += dx;
         y += dy;
         yaw -= dt; // TODO: figure out why this sign is flipped

         old_qcount = qcount;
      }

      // odom position in odom frame
      odom.append(x);
      odom.append(y);
      odom.append(yaw);
      // odom: total of 5 floats; 4*5 = 20 bytes
      odom.finish();
      publish(odom);
   }

   // IMU and GPS loop; run at 20Hz.
   // run at a time when the odometry calculations aren't running
   if( ticks % 50 == 24 ) {
      imu_read(); // read the IMU
   }
}
