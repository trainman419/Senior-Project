/* Atmega2560 PWM Library

   Author: Austin Hendrix
   2010-12-28

   These functions are for PWM generation from the 16-bit timers, 1, 3, 4 and 5

   
   */

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint16_t * timer_base[] = { 
   0,
   &TCNT1,
   0,
   &TCNT3,
   &TCNT4,
   &TCNT5,
};

#define A 0
#define B 1
#define C 2

#define TCNT_OFF 0
#define ICR_OFF 1
#define OC_OFF 2

volatile uint8_t * tccr_base[] = {
   0,
   &TCCR1A,
   0,
   &TCCR3A,
   &TCCR4A,
   &TCCR5A,
};

/* initialize PWM for a particular pin
 *
 * pins should be one of the symbolic constants defined in pwm.h
 *
 * low nibble:   which output compare to set
 * upper nibble: which timer
 */
uint8_t pwm_init(uint8_t pin) {
   uint8_t sreg;

   uint8_t timer = (pin & 0xF0) >> 4;
   uint8_t oc = (pin & 0x0F) - 0x0A; // TODO: eliminate this subtration by re-writing the table


   if( timer >= sizeof(timer_base) ) {
      return 1;
   }

   if( timer_base[timer] == 0 ) {
      return 2;
   }

   if( oc > C ) {
      return 3;
   }

   sreg = SREG;
   cli(); // disable interrupts

   // TODO: set up phase-correct PWM

   SREG = sreg;
   return 0;
}

/* disable pwm for ... ? */
void pwm_off(uint8_t timer) {
}

/* set the duty cycle for pwm */
void pwm_set_duty(uint8_t timer, uint16_t duty) {
}

/* set the timer frequency */
void pwm_set_freq(uint8_t timer, uint16_t freq) {
}
