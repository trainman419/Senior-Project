/* power.h
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

/* read charge of electronics battery. roughly 0-100 */
uint8_t main_battery(); // TODO: implement this

/* read charge of motor battery. roughly 0-100 */
uint8_t motor_battery(); // TODO: implement this
