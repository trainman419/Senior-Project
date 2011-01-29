/* wheelmon.h
   header file for the wheel monitor process

   Author: Austin Hendrix
 */

#include <stdint.h>

#ifndef WHEELMON_H
#define WHEELMON_H

extern volatile int16_t lspeed; /* left wheel speed (Hz) */
extern volatile int16_t rspeed; /* right wheel speed (Hz) */
extern volatile uint16_t lcount; /* left wheel count, revolutions */
extern volatile uint16_t rcount; /* right wheel count, revolutions */

extern volatile int16_t qspeed; /* quaderature encoder speed */
extern volatile uint16_t qcount; /* quaderature encoder speed */

void wheelmon();

#endif
