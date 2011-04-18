/* speedman.h
   Header file for the speed manager function and associated globals

   Author: Austin Hendrix
 */

#include <stdint.h>

#ifndef SPEEDMAN_H
#define SPEEDMAN_h

extern volatile int16_t power;
extern volatile int16_t target_speed;

void speedman();

#endif
