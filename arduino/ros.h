/*
 * ROS Hardware class implementation for my avr/RTOS platform
 *
 * Author: Austin Hendrix
 */

#ifndef ros_h
#define ros_h

#include <ros/ros_impl.h>

namespace ros {
   class AvrHardware {
      public:
         // initialize
         void init();

         // read a byte
         int read();

         // write some bytes
         void write(uint8_t * data, int len);

         // time?
         unsigned long time();
   };

   typedef NodeHandle_<AvrHardware> NodeHandle;
}

#endif
