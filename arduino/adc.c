#include <avr/io.h>
#include <avr/interrupt.h>
#include "system.h"

/* initialize the ADC */
void adc_init() {
   // select AVCC as input voltage reference:
   //  REFS1:0 = 01; see page 289
   ADMUX |= (1 << 6);
   ADMUX &= ~(1 << 7);

   // enable ADC
   ADCSRA |= (1 << 7);

   // disable power reduction for ADC
   PRR0 &= ~1;

   // no conversion, disable auto trigger, disable interrupt; see page 293
   ADCSRA &= ~( (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3));
   // ADC clock target between 50kHz and 200kHz. Input clock 16MHz
   // maximum prescaler 128, ADC clock of 125kHz
   // set ADC prescaler to 128; 0x7
   ADCSRA |= 0x07;


   // disable digital input on all 16 analog pins
   //  this saves some power, see page 295
   //DIDR0 = 0xFF;
   //DIDR2 = 0xFF;
}

/* read from the specified ADC channel */
uint16_t adc_read(uint8_t channel) {
   // with the given prescaler rate, a conversion will take 1664 clocks, or
   // about 0.1mS. SLOW!
   uint16_t res = 0;
   
   // lower 3 bits of channel in ADCSRA2:0
   ADMUX &= ~(0x7);
   ADMUX |= channel & 0x07;
   // upper bit of channel in ADCSR3
   ADCSRB &= ~(0x08);
   ADCSRB |= (channel & 0x08);
   // set differential bits (4:3) to zero
   ADMUX &= ~( (1 << 3) | (1 << 4));

   // initiate conversion
   ADCSRA |= (1 << 6);

   // wait for conversion to complete
   while( ADCSRA & (1 << 6) );

   res = ADC;

   return res;
}
