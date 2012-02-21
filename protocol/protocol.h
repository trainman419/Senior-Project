/* protocol.h
 *
 * A cross-platofrm AVR and x86 serial communication library.
 *
 * Author: Austin Hendrix
 */

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

class Packet {
   private:
      //const static uint8_t PACKET_SZ = N;

      char * const buffer;
      const uint8_t buf_sz;
      uint8_t sz;
      uint8_t idx;

      const static char esc = 0x1B;

   public:
      // construct a packet, set first byte to type
      Packet(char type, unsigned char s, char * b) : 
         buffer(b), buf_sz(s), sz(1), idx(1)
      { sz = 1; buffer[0] = type; }

      // construct a packet from a buffer
      Packet(char * in, unsigned char in_sz);

      // directly append a character, so we can use packets as a buffer
      void input(char c);

      // append various data types to a packet
      void append(uint8_t  c);
      void append(int8_t   c);
      void append(uint16_t s);
      void append(int16_t  s);
      void append(uint32_t i);
      void append(int32_t  i);
      void append(uint64_t i);
      void append(int64_t  i);
      void append(float    f);
      void append(double   d);

      //void append(volatile uint16_t & s);
      //void append(volatile int16_t  & s);

      // read various types from a packet
      unsigned char  readu8();
      signed char    reads8();
      unsigned short readu16();
      signed short   reads16();
      uint32_t       readu32();
      int32_t        reads32();
      float          readfloat();

      // get data for sending packet
      const char * outbuf() { return buffer; }
      unsigned char outsz() { return sz; }

      // utility methods
      void finish();
      void reset() { sz = 1; idx = 1; }
};

#endif
