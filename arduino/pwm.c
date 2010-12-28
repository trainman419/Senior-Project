/* Atmega2560 PWM Library

   Author: Austin Hendrix
   2010-12-28
   */

#include <avr/io.h>
#include <avr/interrupt.h>

#define TIMER_BASE 

void pwm_init(unsigned char timer) {
   unsigned char sreg;

   sreg = SREG;
   cli(); // disable interrupts


   SREG = sreg;
   return;
}

void pwm_off(unsigned char timer) {
}

void pwm_set_duty(unsigned char timer, unsigned char duty) {
}

void pwm_set_freq(unsigned char timer, unsigned int freq) {
}
