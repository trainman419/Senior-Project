/*
 * Implementation of ROS hardware interface for my platform
 *
 * Author: Austin Hendrix
 */

#include "ros.h"
#include "serial.h"

namespace ros {

   // initialize
   void AvrHardware::init() {
   }

   // read a byte
   int AvrHardware::read() {
      return 0;
   }

   // write some bytes
   void AvrHardware::write(uint8_t * data, int len) {
   }

   // time?
   unsigned long AvrHardware::time() {
      return 0;
   }
}
