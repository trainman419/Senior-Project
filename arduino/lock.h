/* lock.h
 * 
 * Common locking functions
 *
 * Author: Austin Hendrix
 */

#ifndef LOCK_H
#define LOCK_H

#include <avr/interrupt.h>
#include "system.h"

static inline void acquire_lock(uint8_t * l) {
   uint8_t success = 0;
   while(!success) {
      while(*l) yeild();
      // disable interrupts
      cli();
      if( !*l ) {
         *l = current_pid+1;
         success = 1;
      }
      sei();
   }
}

static inline void release_lock(uint8_t * l) {
   *l = 0;
}

#endif
