/* speedman.h
   Header file for the speed manager function and associated globals

   Author: Austin Hendrix
 */

#ifndef SPEEDMAN_H
#define SPEEDMAN_h

// speed controller mode
#define M_OFF 0
#define M_FORWARD 1
#define M_BRAKE 2
#define M_REVERSE 3
extern u08 mode;

extern volatile s16 power;
extern volatile s16 target_speed;

void speedman();

#endif
