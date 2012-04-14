/* estop.h
 *
 * Emergency-stop implementation
 *
 * Author: Austin Hendrix
 */

#ifndef ESTOP_H
#define ESTOP_H

#include <stdint.h>

// initialize the E-stop hardware
void estop_init();

// return true if the robot is stopped
uint8_t estop();

#endif
