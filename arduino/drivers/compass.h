/* compass.h
 *
 * header file for assembly compass routines.
 *
 * Author: Austin Hendrix
 */

#include <stdint.h>

#ifndef COMPASS_H
#define COMPASS_H

/* heading sturcture */
struct heading
{
	int16_t x;
	int16_t y;
};

/* take compass reading */
void compass();
struct heading compass_poll();

/* initialize compass */
void compass_init();

#endif
