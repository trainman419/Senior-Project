/* power.c
 *
 * Robot power management functions, including:
 *  5V main regulator enable/disable
 *  full power off
 *  battery monitoring
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>
#include "power.h"
#include "adc.h"

/* enable 5V regulator */
void pwr_on() {
   // arduino mega pin 37; PC0
   DDRC |= 1;
   PORTC |= 1;
}

/* disable 5V regulator */
void pwr_sleep() {
   // arduino mega pin 37: PC0
   DDRC |= 1;
   PORTC &= ~1;
}

/* full system shutdown. there is no going back! */
void pwr_off() {
   // arduino mega pin 48: PL1
   DDRL |= (1 << 1);
   PORTL |= (1 << 1);
}

/* initialize ADCs for reading battries */
void battery_init() {
   adc_init();
}

/* read charge of electronics battery. roughly 0-100 */
/*
 * Measurement notes:
 *    9.2V: 134
 *    9.15V: 133
 *    9.1V: 132
 *    8.95V: 130
 *    8.90V: 129
 *    8.85V: 128
 *    8.77V: 127
 *    8.70V: 126
 *    8.60V: 124
 *    7.05V: 099 (still stable; not heavily tested)
 *
 * Run-time test: 2.5 hours!
 */
uint8_t main_battery() {
   // TODO: implement this properly
   uint16_t adc = adc_read(7);
   return adc >> 2;
}

/* read charge of motor battery. roughly 0-100 */
/*
 * Measurement notes:
 *    9.4V: 149
 *    0.5V: 025
 */
uint8_t motor_battery(){
   // TODO: implement this properly
   // this is likely to share A LOT of code with main_battery()
   uint16_t adc = adc_read(15);

   return adc >> 2;
}
