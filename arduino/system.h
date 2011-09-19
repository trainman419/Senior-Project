/* system.h
 * header file for simple operating system (system.s) for ATMega32/Polybot
 *
 * defines system(), system_init() and yeild() functions
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

char system( void (*)(void), unsigned char sched, unsigned char pri);

void system_init(void);

void yeild(void);

extern char current_pid;
extern char num_pids;
extern uint32_t ticks;

struct pid_entry {
   unsigned char spl;
   unsigned char sph;
   unsigned char schedule;
   unsigned char last;
   unsigned char priority;
};

void schedule(unsigned char sched);

void priority(unsigned char pri);

extern struct pid_entry process_table[4];

#endif
