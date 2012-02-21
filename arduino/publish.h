/*
 * publish.h
 *
 * A wrapper around my protocol to make sending messages easier.
 *
 * Author: Austin Hendrix
 */

#ifndef PUBLISH_H
#define PUBLISH_H

#include "protocol.h"
extern "C" {
#include "drivers/serial.h"
};

/*
template<int SZ> void publish(Packet<SZ> & p) {
   int sz = p.outsz();
   tx_buffer(BRAIN, (uint8_t*)p.outbuf(), sz);
}
*/

// experimental publisher class; wraps the Packet class
// targeted to my AVR
template<uint8_t SZ>
class Publisher {
   private:
      uint16_t brain_sz;
      uint16_t bt_sz;
      char buffer[SZ];
      Packet p;

   public:
      Publisher(char topic) : p(topic, (uint8_t)SZ, buffer) {
      }

      void reset() {
         while(brain_sz > 0);
         while(bt_sz > 0);
         p.reset();
      }

      void finish() {
         p.finish();
         brain_sz = p.outsz();
         tx_buffer(BRAIN, (const uint8_t *)p.outbuf(), &brain_sz);

         //bt_sz = p.outsz();
         //tx_buffer(BT, (const uint8_t *)p.outbuf(), &bt_sz);
      }

      template<class T> void append(T t) {
         p.append(t);
      }
};

#endif
