/*
 * Implementation of ROS hardware interface for my platform
 *
 * Author: Austin Hendrix
 */

#include "ros.h"
extern "C" {
#include "serial.h"
#include "interrupt.h"
#include <avr/interrupt.h>
};

namespace ros {

   // initialize
   void AvrHardware::init() {
   }

   // read a byte
   int AvrHardware::read() {
      if(rx_ready(BRAIN)) {
         return rx_byte(BRAIN);
      } else {
         return -1;
      }
   }

   // write some bytes
   void AvrHardware::write(Out & out) {
      if( out.pos == out.size ) 
         tx_buffer(BRAIN, out.buffer + out.start, out.pos);
   }

   // time?
   unsigned long AvrHardware::time() {
      return ticks;
   }

   // get an Out object for some amount of space
   AvrHardware::Out AvrHardware::getSpace(uint16_t size) {
      // FIXME: check available space before start
      cli();
      uint16_t s = sz;
      sz = (sz + size) % BUFSZ;
      sei();
      return AvrHardware::Out(buffer, s, size);
   }

   void AvrHardware::Out::write(unsigned char c) {
      buffer[(start + pos) % AvrHardware::BUFSZ] = c;
      ++pos;
   }
}
