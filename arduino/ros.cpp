/*
 * Implementation of ROS hardware interface for my platform
 *
 * Author: Austin Hendrix
 */

#include "ros.h"
extern "C" {
#include "drivers/serial.h"
#include "interrupt.h"
#include "drivers/led.h"
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
   uint8_t AvrHardware::write(Out & out) {
      if( out.pos == out.size && out.fail == 0 ) {
         tx_buffer(BRAIN, out.buffer, out.pos);
         return 0;
      }
      led_on();
      return 1;
   }

   // time?
   unsigned long AvrHardware::time() {
      return ticks;
   }

   // get an Out object for some amount of space
   //  returns a zero-size object if memory allocation failed.
   AvrHardware::Out AvrHardware::getSpace(uint16_t size) {
      uint8_t sreg = SREG;
      cli();
      uint8_t * b = (uint8_t*) malloc(size);
      uint16_t s = size;
      if(!b) {
        s = 0;
      }
      SREG = sreg;
      return AvrHardware::Out(b, s);
   }

   void AvrHardware::Out::write(unsigned char c) {
      if( pos < size ) {
         buffer[pos] = c;
         ++pos;
      } else {
        fail = 1;
        led_on();
      }
   }
}
