/*
 * Header for new interrupt-based management.
 *
 */

#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "ros.h"

void interrupt_init(void);

extern volatile uint32_t ticks;

extern ros::Publisher odom_pub;

#endif
