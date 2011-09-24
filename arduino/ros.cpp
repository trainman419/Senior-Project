/*
 * Implementation of ROS hardware interface for my platform
 *
 * Author: Austin Hendrix
 */

#include "ros.h"
extern "C" {
#include "serial.h"
#include "interrupt.h"
};

namespace ros {

   // initialize
   void AvrHardware::init() {
   }

   // read a byte
   int AvrHardware::read() {
      while(!rx_ready(BRAIN));
      return rx_byte(BRAIN);
   }

   // write some bytes
   void AvrHardware::write(uint8_t * data, int len) {
      while(sz != 0);
      sz = len;
      tx_buffer(BRAIN, data, &sz);
   }

   // time?
   unsigned long AvrHardware::time() {
      return ticks;
   }
}
