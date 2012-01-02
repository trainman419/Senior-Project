/*
 * ROS Hardware class implementation for my avr/RTOS platform
 *
 * Author: Austin Hendrix
 */

#ifndef ros_h
#define ros_h

#include <ros/node_handle.h>

#define BUFSZ 4096

namespace ros {
   class AvrHardware {
      public:
         AvrHardware() {
         }
         // initialize
         void init();

         // read a byte
         int read();

         // class to buffer output writes
         class Out : public ros::Out {
            private:
               uint8_t * buffer;
               uint16_t size;
               uint16_t pos;
               uint8_t fail;
               Out(uint8_t * b, uint16_t sz) : buffer(b), 
                  size(sz), pos(0), fail(0) {}

               friend class AvrHardware;

            public:
               virtual void write(unsigned char c);
         };

         // write an Out object
         uint8_t write(Out & o);

         // get an Out object for some amount of space
         Out getSpace(uint16_t size);

         // time?
         unsigned long time();
   };

   //      maximum sizes for:      sub, pub, in, out
   typedef NodeHandle_<AvrHardware, 25, 25, 512, BUFSZ> NodeHandle;
}

#endif
