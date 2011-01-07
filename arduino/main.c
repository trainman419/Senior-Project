/* main.c: the main entry point for my arduino control code.

   Target Board: Arduino Mega 2560
   Target Processor: Atmel ATMega2560

   Author: Austin Hendrix
 */

#include <avr/io.h>
#include "pwm.h"
#include "motor.h"

#define CLK 16000

int main() {
   int i;
   int j;

   // motor control
   int dir = 0;
   int speed = 0;
   DDRB |= 1 << 7;
   motor_init();

   int duty = 50;
   // LED pwm setup
   pwm_init(PWM13);
   pwm_set_freq(1, 200);
   pwm_set_duty(PWM13, 0.5);
   
   while(1) {
      for(j=0;j<80; j++)
         for(i=0;i<CLK; i++);

      duty += 10;
      if( duty > 100 ) duty = 0;

      pwm_set_duty(PWM13, ((float)duty) / 100.0);

      if( dir ) speed += 10;
      else speed -= 10;
      if( speed == 100 ) dir = 0;
      if( speed == -100 ) dir = 1;
      motor_speed(speed);
   }

   // loop forever instead of exiting
   while(1);
}
