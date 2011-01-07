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

   int duty = 50;

   DDRB |= 1 << 7;
   DDRB |= 1 << 6;

   pwm_init(PWM12); // LED pwm
   pwm_set_freq(1, 200);
   pwm_set_duty(PWM12, 0.5);

   while(1) {
      for(j=0;j<80; j++)
         for(i=0;i<CLK; i++);

      duty += 10;
      if( duty > 100 ) duty = 0;

      pwm_set_duty(PWM12, ((float)duty) / 100.0);
   }

   // loop forever instead of exiting
   while(1);
}
