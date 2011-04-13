/* protocol.h
 *
 * A cross-platofrm AVR and x86 serial communication library.
 *
 * Author: Austin Hendrix
 */

#ifndef PACKET_H
#define PACKET_H

template <int PACKET_SZ>
class Packet {
   private:
      //const static uint8_t PACKET_SZ = N;

      char buffer[PACKET_SZ];
      unsigned char sz;
      unsigned char idx;

      const static char esc = 0x1B;

   public:
      // construct a packet, set first byte to type
      Packet(char type) : sz(1), idx(1) { sz = 1; buffer[0] = type; }

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

// constructors
template <int PACKET_SZ>
Packet<PACKET_SZ>::Packet(char * in, uint8_t in_sz) {
   uint8_t i;
   for( i=0; i<in_sz && i<PACKET_SZ; i++) {
      buffer[i] = in[i];
   }
   sz = in_sz<PACKET_SZ?in_sz:PACKET_SZ;
   idx = 1;
}

// directly append a character to the internal buffer
template <int PACKET_SZ>
void Packet<PACKET_SZ>::input(char c) {
   if( sz < PACKET_SZ ) {
      buffer[sz++] = c;
   }
}

// append methods
template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(uint8_t c) {
   if( sz < PACKET_SZ ) {
      if( c != '\r' && c != esc ) {
         buffer[sz++] = c;
      } else {
         buffer[sz++] = esc;
         buffer[sz++] = c ^ esc;
      }
   }
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(int8_t c) {
   append((uint8_t) c);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(uint16_t s) {
   uint8_t tmp;
   tmp = s & 0xFF;
   append(tmp);
   tmp = (s >> 8) & 0xFF;
   append(tmp);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(int16_t s) {
   append((uint16_t) s);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(uint32_t i) {
   uint8_t tmp;
   tmp = i & 0xFF;
   append(tmp);
   i >>= 8;
   tmp = i & 0xFF;
   append(tmp);
   i >>= 8;
   tmp = i & 0xFF;
   append(tmp);
   i >>= 8;
   tmp = i & 0xFF;
   append(tmp);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(int32_t i) {
   append((uint32_t)i);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(uint64_t i) {
   uint8_t tmp;

   tmp = i & 0xFF;
   append(tmp);
   tmp >>= 8;

   tmp = i & 0xFF;
   append(tmp);
   tmp >>= 8;

   tmp = i & 0xFF;
   append(tmp);
   tmp >>= 8;

   tmp = i & 0xFF;
   append(tmp);
   tmp >>= 8;

   tmp = i & 0xFF;
   append(tmp);
   tmp >>= 8;

   tmp = i & 0xFF;
   append(tmp);
   tmp >>= 8;

   tmp = i & 0xFF;
   append(tmp);
   tmp >>= 8;

   tmp = i & 0xFF;
   append(tmp);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(int64_t i) {
   append((uint64_t)i);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(float f) {
   uint32_t t = *((uint32_t*)(&f));
   append(t);
}

template <int PACKET_SZ>
void Packet<PACKET_SZ>::append(double d) {
   uint64_t t = *((uint64_t*)(&d));
   append(t);
}

/*void Packet::append(volatile int16_t & s) {
   append((int16_t)s);
}

void Packet::append(volatile uint16_t & s) {
   append((uint16_t)s);
}*/

// read methods
template <int PACKET_SZ>
unsigned char Packet<PACKET_SZ>::readu8() {
   unsigned char tmp;
   if(idx < sz) {
      tmp = buffer[idx++];
      if( tmp != esc ) {
         return tmp;
      } else {
         if( idx < sz ) {
            tmp = buffer[idx++] ^ esc;
            return tmp;
         } else {
            return 0;
         }
      }
   } else {
      return 0;
   }
}

template <int PACKET_SZ>
signed char Packet<PACKET_SZ>::reads8() {
   return readu8();
}

template <int PACKET_SZ>
unsigned short Packet<PACKET_SZ>::readu16() {
   unsigned char lo = readu8();
   unsigned char hi = readu8();
   return lo | (hi << 8);
}

template <int PACKET_SZ>
signed short Packet<PACKET_SZ>::reads16() {
   return readu16();
}

template <int PACKET_SZ>
uint32_t Packet<PACKET_SZ>::readu32() {
   uint32_t bytes[4];
   uint32_t res;
   bytes[0] = readu8();
   bytes[1] = readu8();
   bytes[2] = readu8();
   bytes[3] = readu8();
   res = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
   return res;
}

template <int PACKET_SZ>
int32_t Packet<PACKET_SZ>::reads32() {
   return (int32_t)readu32();
}

template <int PACKET_SZ>
float Packet<PACKET_SZ>::readfloat() {
   uint32_t tmp = readu32();
   return *((float*)&tmp);
}

// utility functions

// do any final prep before transmitting a packet
template <int PACKET_SZ>
void Packet<PACKET_SZ>::finish() {
   buffer[sz++] = '\r';
}

#endif
