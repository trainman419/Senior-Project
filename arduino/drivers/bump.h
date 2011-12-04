/*
 * bump.h
 *
 * interface functions for bump sensors
 *
 * Author: Austin Hendrix
 */

#ifndef BUMP_H
#define BUMP_H

#include <stdint.h>

void bump_init(void);

// get the state of the bump sensors
uint8_t bumpL(void);
uint8_t bumpR(void);

// return true if either sensor is pressed
uint8_t bump(void);

#endif
