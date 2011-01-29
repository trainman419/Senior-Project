/* speedman.h
   Header file for the speed manager function and associated globals

   Author: Austin Hendrix
 */

#include <stdint.h>

#ifndef SPEEDMAN_H
#define SPEEDMAN_h

// speed controller mode
#define M_OFF 0
#define M_FORWARD 1
#define M_BRAKE 2
#define M_REVERSE 3
extern uint8_t mode;

extern volatile int16_t power;
extern volatile int16_t target_speed;

void speedman();

#endif
