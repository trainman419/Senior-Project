/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#include <avr/io.h>
#include "pwm.h"

#define CLK 16000

int main() {
   int i;
   int j;

   int duty = 0;

   DDRB |= 1 << 7;
   DDRB |= 1 << 6;

   uint8_t init = pwm_init(PWM12); // LED pwm
   uint8_t freq = pwm_set_freq(1, 200);
   uint8_t d = pwm_set_duty(PWM12, 0.5);

   // check for errors and flash the LED
   if( init || freq || d ) {
      uint8_t f = 8;
      if( d ) f = 4;
      if( freq ) f = 2;
      if( init ) f = 1;

      while(1) {
         for(j=0;j<f; j++)
            for(i=0;i<1600; i++);

         PORTB &= ~(1 << 7);

         for(j=0;j<f; j++)
            for(i=0;i<1600; i++);

         PORTB |= 1 << 7;
      }
   }

   while(1) {
      for(j=0;j<10; j++)
         for(i=0;i<CLK; i++);

      duty++;
      if( duty >= 100 ) duty = 0;

      pwm_set_duty(PWM12, ((float)duty) / 100.0);
   }

   // loop forever instead of exiting
   while(1);
}
