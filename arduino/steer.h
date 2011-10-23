/*
 * Abstraction for steering mechanism; converts turning radius to steering
 * setting and back.
 *
 * Author: Austin Hendrix
 */

#ifndef STEER_H
#define STEER_H

#include <stdint.h>

// convert radius to positive steering setting
int8_t radius2steer(float r);

// convert steering setting (+ or -) to positive radius
float steer2radius(int8_t s);

#endif
