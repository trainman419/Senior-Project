#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * FIXME: support more than one servo in the interrupt.
 * FIXME: support a servo attached to some pin other than PC1
 */

/* initialize the timer that generates servo interrupts */
void servo_init() {
   // use a random 16-bit timer; any will do
   // timer1
   // WGM mode: Fast PWM ICRn (1110)
   //  clears timer when it matches ICR

   // normal mode; no output generation; WGM bits 1:0 = 00
   TCCR1A = 0x02;

   // normal mode, prescaler divide by 64. see page 161; WGM bits 3:2 = 11
   TCCR1B = (0x18) | (0x3);

   // don't force output compare
   TCCR1C = 0x00;

   // interrupt enable. enable OC1A and OVR interrupts, page 166
   TIMSK1 = 0x03;

   // set to overflow every 100mS
   ICR1 = 2499;
   // this means that 250 timer steps equals 1mS
}

volatile uint8_t * servo_port[8];
uint8_t servo_pin[8];

// a note: both of these interrupts should take about a dozen instructions 
//  to execute, with all of the output hardcoding I've done

// compare match interrupt
ISR(TIMER1_COMPA_vect) {
   /*if( servo_port[0] )
      *servo_port[0] &= ~(1 << servo_pin[0]);
      */
   PORTC &= ~(1 << 1);
}

// overflow interrupt
ISR(TIMER1_OVF_vect) {
   /*if( servo_port[0] )
      *servo_port[0] |= (1 << servo_pin[0]);
      */
   PORTC |= (1 << 1);
}

/* map a particular pin to a servo index */
void servo_map(uint8_t servo, volatile uint8_t * port, uint8_t pin) {
   if( servo < 8 ) {
      servo_port[servo] = port;
      servo_pin[servo] = pin;
   }
}

/* set servo to the desired position */
void servo_set(uint8_t servo, uint8_t set) {
   // 250 steps = 1mS
   //
   // formula: (set - 127) + 375
   uint16_t out = set + 248;

   OCR1A = out;
}
