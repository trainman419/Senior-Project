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

   pwm_init(5);

   DDRB |= 1 << 7;

   while(1) {
      for(j=0;j<10; j++)
         for(i=0;i<CLK; i++);

      PORTB &= ~(1 << 7);

      for(j=0;j<10; j++)
         for(i=0;i<CLK; i++);
      PORTB |= 1 << 7;
   }

   // loop forever instead of exiting
   while(1);
}
