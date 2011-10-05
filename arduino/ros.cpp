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
      if(rx_ready(BRAIN)) {
         return rx_byte(BRAIN);
      } else {
         return -1;
      }
   }

   // write some bytes
   void AvrHardware::write(uint8_t * data, uint16_t len) {
      tx_buffer(BRAIN, data, len);
   }

   // time?
   unsigned long AvrHardware::time() {
      return ticks;
   }
}
