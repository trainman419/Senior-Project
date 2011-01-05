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

   pwm_init(PWM13); // LED pwm
   pwm_set_freq(0, 200);
   pwm_set_duty(PWM13, 0.0);

   DDRB |= 1 << 7;

   while(1) {
      for(j=0;j<10; j++)
         for(i=0;i<CLK; i++);

      duty++;
      if( duty >= 100 ) duty = 0;

      pwm_set_duty(PWM13, duty / 100.0);
      /*PORTB &= ~(1 << 7);

      for(j=0;j<10; j++)
         for(i=0;i<CLK; i++);
      PORTB |= 1 << 7;*/
   }

   // loop forever instead of exiting
   while(1);
}
