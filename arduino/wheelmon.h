/* wheelmon.h
   header file for the wheel monitor process

   Author: Austin Hendrix
 */

#ifndef WHEELMON_H
#define WHEELMON_H

extern volatile s16 lspeed; /* left wheel speed (Hz) */
extern volatile s16 rspeed; /* right wheel speed (Hz) */
extern volatile u16 lcount; /* left wheel count, revolutions */
extern volatile u16 rcount; /* right wheel count, revolutions */

extern volatile s16 qspeed; /* quaderature encoder speed */
extern volatile u16 qcount; /* quaderature encoder speed */

void wheelmon();

#endif
