/*
 * ROS Hardware class implementation for my avr/RTOS platform
 *
 * Author: Austin Hendrix
 */

#ifndef ros_h
#define ros_h

#include <ros/node_handle.h>

namespace ros {
   class AvrHardware {
      public:
         static const uint16_t BUFSZ = 2048;

         AvrHardware() : sz(0) {}
         // initialize
         void init();

         // read a byte
         int read();

         // class to buffer output writes
         class Out : public ros::Out {
            private:
               uint8_t * buffer;
               uint16_t start;
               uint16_t size;
               uint16_t pos;
               Out(uint8_t * b, uint16_t s, uint16_t sz) : buffer(b), start(s),
                  size(sz), pos(0) {}

               friend class AvrHardware;

            public:
               virtual void write(unsigned char c);
         };

         // write an Out object
         void write(Out & o);

         // get an Out object for some amount of space
         Out getSpace(uint16_t size);

         // time?
         unsigned long time();
      private:
         uint16_t sz;

         uint8_t buffer[BUFSZ];
   };

   //      maximum sizes for:      sub, pub, in, out
   typedef NodeHandle_<AvrHardware, 25, 25, 512, 512> NodeHandle;
}

#endif
