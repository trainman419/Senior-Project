/*
 * Abstraction for steering mechanism; converts turning radius to steering
 * setting and back.
 *
 * Author: Austin Hendrix
 */

#ifndef STEER_H
#define STEER_H

#include <stdint.h>

int8_t radius2steer(float r);

float steer2radius(int8_t s);

#endif
