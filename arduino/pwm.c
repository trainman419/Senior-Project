/* Atmega2560 PWM Library

   Author: Austin Hendrix
   2010-12-28

   These functions are for PWM generation from the 16-bit timers, 1, 3, 4 and 5

   These set up the phase-correct, non-inverted PWM mode.
   Using the ICRn registers for TOP

WGMn3: 1 (TCCRnB bit 4)
WGMn2: 0 (TCCRnB bit 3)
WGMn1: 1 (TCCRnA bit 1)
WGMn0: 0 (TCCRnA bit 0)

write TCCRnB bits 6 and 7 to 0
TCCRnB bits 2 to 0 set prescaler (datasheet page 161)

To enable output, set
COMnx1 = 1 (TCCRnA bit 2x + 3)
COMnx0 = 0 (TCCRnA bit 2x + 2)

Also set direction bit for output pin
DDRx = ???

Set TIMSKn to 0 (disable all interrupts)

frequency equation:
f = clk / (2 * N * TOP)
N = prescaler
TOP = top value, ICRn
   
   */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "pwm.h"

#define F_CPU 16000000

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

inline uint8_t check_pin(uint8_t pin) {
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
   return 0;
}

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

   uint8_t ck = check_pin(pin);
   if( ck ) return ck;

   sreg = SREG;
   cli(); // disable interrupts

   // set up phase-correct PWM
   volatile uint8_t * tccr = tccr_base[timer];

   // set WGMn1 and COMnx1 
   tccr[A] |= (2 | (1 << (oc*2 + 3)));
   // clear WGMn0 and COMnx0
   tccr[A] &= ~(1 | (1 << (oc*2 + 2)));

   // set WGMn3
   tccr[B] |= 1;
   // clear WGMn2 and bits 6 and 7
   tccr[B] &= ~( (1 << 3) | (1 << 6) | (1 << 7));

   // leave prescaler bits alone

   SREG = sreg;
   return 0;
}

/* disable pwm output for a particular pin
 *
 * same input as for pwm_init()
 */
void pwm_off(uint8_t timer) {
}

/* set the duty cycle for pwm 
 * 
 * timer: same format as for pwm_init
 * duty: % duty cycle or absolute OCRn value?
 *   % duty cycle means consumers don't have to think
 *   OCR values give the most control
 */
/*
 * percentage as float?
 * store precentage so that we can maintain duty cycle when changing
 *   frequency?
 */
float pwm_duty[15] = {
   0.0, 0.0, 0.0, 
   0.0, 0.0, 0.0,
   0.0, 0.0, 0.0,
   0.0, 0.0, 0.0,
   0.0, 0.0, 0.0,
};

uint8_t pwm_set_duty(uint8_t pin, float duty) {
   uint8_t timer = (pin & 0xF0) >> 4;
   uint8_t oc = (pin & 0x0F) - 0x0A; // TODO: remove subtraction

   uint8_t ck = check_pin(pin);
   if( ck ) return ck;

   volatile uint16_t * base = timer_base[timer];

   uint16_t icr = base[ICR_OFF];

   uint16_t ocr = 0;
   ocr = icr * duty; // this is EXPENSIVE!
   if( ocr > icr ) ocr = icr;

   base[OC_OFF + oc] = ocr;
   pwm_duty[timer*3 + oc] = duty;

   return 0;
}

#define ICR_MAX 65536

/* set the timer frequency */
/* for prescaler setup, see page 161 of the datasheet */
uint8_t pwm_set_freq(uint8_t timer, uint16_t freq) {
   if( timer >= sizeof(timer_base) ) {
      return 1;
   }

   if( timer_base[timer] == 0 ) {
      return 2;
   }

   volatile uint16_t * icr = timer_base[timer] + ICR_OFF;

   uint32_t base = F_CPU;

   // number of clock ticks per pwm cycle
   base /= freq;

   // timer count steps (ticks/2 since we're using phase-correct mode)
   base /= 2;

   // we want the smallest prescaler, so we the the most resolution
   uint8_t prescale = 1; // no prescaler

   // check if we need /8 prescaler
   if( base > ICR_MAX ) {
      prescale++;
      base /= 8;
   }

   // check if we need /64 prescaler
   if( base > ICR_MAX ) {
      prescale++;
      base /= 8;
   }

   // check if we need /256 prescaler
   if( base > ICR_MAX ) {
      prescale++;
      base /= 4;
   }

   // check if we need final /1024 prescaler
   if( base > ICR_MAX ) {
      prescale++;
      base /= 4;
   }

   // set up our compare register
   *icr = base - 1;

   // set our prescaler
   volatile uint8_t * tccr = tccr_base[timer];

   tccr[C] &= ~(0x7);
   tccr[C] |= 0x7 & prescale;

   return 0;
}
