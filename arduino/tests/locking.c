#define F_CPU 16000000UL

#include <stdlib.h>
#include <avr/interrupt.h>
#include <drivers/serial.h>
#include <util/delay.h>
#include <drivers/led.h>

#define LEN 26
char * s1 = "`zyxwvutsrqponmlkjihgfedcba{|}~?";
char * s2 = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_";

ISR(TIMER0_OVF_vect) {
   int i;
   uint8_t * b1 = (uint8_t*) malloc(LEN);
   if( b1 ) {
     for( i=0; i<LEN; i++ ) b1[i] = s1[i];

     tx_buffer(BRAIN, b1, LEN);
   } else {
     led_on();
   }
}

int main() {
   int i;
   int reset = 0;
   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);

   // set up timer interrupts
   // fast PWM mode; interrupt and reset when counter equals OCR0A
   // prescalar 64
   TCCR0A = (1 << WGM01 | 1 << WGM00);
   TCCR0B = (1 << WGM02 | 1 << CS02 | 1 << CS00);
   // interrupt on "overflow" (counter match)
   TIMSK0 = (1 << TOIE0);
   OCR0A  = 249; // 250 counts per tick

   _delay_ms(1000);

   sei();

   while(1) {
     cli();
     uint8_t * b2 = (uint8_t*) malloc(LEN);
     sei();
     if( b2 ) {
       for( i=0; i<LEN; i++ ) b2[i] = s2[i];
       tx_buffer(BRAIN, b2, LEN);
     } else {
       led_on();
     }
     _delay_ms(100);
     reset++;
     if( reset > 20 ) led_off();
   }
}
