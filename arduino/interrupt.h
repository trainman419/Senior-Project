/*
 * Header for new interrupt-based management.
 *
 */

#ifndef INTERRUPT_H
#define INTERRUPT_H

void interrupt_init(void);

extern volatile uint32_t ticks;

#endif
