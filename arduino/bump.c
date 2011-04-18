/* 
 * bump.c
 * Implementation of bumper drivers
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>
#include "bump.h"

#define R 0x08
#define L 0x04

void bump_init(void) {
   // set pins as input and enable pull-up
   DDRC &= ~(R | L);
   PORTC |= (R | L);
}

// get the state of the bump sensors
uint8_t bumpL(void) {
   return !(PINC & L);
}

uint8_t bumpR(void) {
   return !(PINC & R);
}

// return true if either sensor is pressed
uint8_t bump(void) {
   return (PINC & (R | L)) != (L | R);
}
