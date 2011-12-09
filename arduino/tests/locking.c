#define F_CPU 16000000UL

#include <stdlib.h>
#include <avr/interrupt.h>
#include <drivers/serial.h>
#include <util/delay.h>

char * s1 = "abcdefghijklmnopqrstuvwxyz";
char * s2 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

ISR(TIMER0_OVF_vect) {
   int i;
   uint8_t * b1 = (uint8_t*) malloc(26);
   for( i=0; i<26; i++ ) b1[i] = s1[i];

   tx_buffer(BRAIN, b1, 26);
}

int main() {
   int i;
   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);

   // set up timer interrupts
   // fast PWM mode; interrupt and reset when counter equals OCR0A
   // prescalar 64
   TCCR0A = (1 << WGM01 | 1 << WGM00);
   TCCR0B = (1 << WGM02 | 1 << CS01 | 1 << CS00);
   // interrupt on "overflow" (counter match)
   TIMSK0 = (1 << TOIE0);
   OCR0A  = 249; // 250 counts per tick

   sei();

   while(1) {
      uint8_t * b2 = (uint8_t*) malloc(26);
      for( i=0; i<26; i++ ) b2[i] = s2[i];
      tx_buffer(BRAIN, b2, 26);
      _delay_ms(1);
   }
}
