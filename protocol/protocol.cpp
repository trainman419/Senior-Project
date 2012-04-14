/* protocol.cpp
 *
 * A cross-platform communication library for AVR and x86.
 *
 * Author: Austin Hendrix
 */

#include "protocol.h"

// constructors
Packet::Packet(char * in, uint8_t in_sz) : buffer(in), buf_sz(in_sz) {
   /*
   uint8_t i;
   for( i=0; i<in_sz && i<buf_sz; i++) {
      buffer[i] = in[i];
   }
   sz = in_sz<buf_sz?in_sz:buf_sz;
   */
   sz = in_sz;
   idx = 1;
}

// directly append a character to the internal buffer
void Packet::input(char c) {
   if( sz < buf_sz ) {
      buffer[sz++] = c;
   }
}

// append methods
void Packet::append(uint8_t c) {
   if( sz < (buf_sz-2) ) {
      if( c != '\r' && c != esc ) {
         buffer[sz++] = c;
      } else {
         buffer[sz++] = esc;
         buffer[sz++] = c ^ esc;
      }
   }
}

void Packet::append(int8_t c) {
   append((uint8_t) c);
}

void Packet::append(uint16_t s) {
   uint8_t tmp;
   tmp = s & 0xFF;
   append(tmp);
   tmp = (s >> 8) & 0xFF;
   append(tmp);
}

void Packet::append(int16_t s) {
   append((uint16_t) s);
}

void Packet::append(uint32_t i) {
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

void Packet::append(int32_t i) {
   append((uint32_t)i);
}

void Packet::append(uint64_t i) {
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

void Packet::append(int64_t i) {
   append((uint64_t)i);
}

void Packet::append(float f) {
   uint32_t t = *((uint32_t*)(&f));
   append(t);
}

void Packet::append(double d) {
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
unsigned char Packet::readu8() {
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

signed char Packet::reads8() {
   return readu8();
}

unsigned short Packet::readu16() {
   unsigned char lo = readu8();
   unsigned char hi = readu8();
   return lo | (hi << 8);
}

signed short Packet::reads16() {
   return readu16();
}

uint32_t Packet::readu32() {
   uint32_t bytes[4];
   uint32_t res;
   bytes[0] = readu8();
   bytes[1] = readu8();
   bytes[2] = readu8();
   bytes[3] = readu8();
   res = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
   return res;
}

int32_t Packet::reads32() {
   return (int32_t)readu32();
}

float Packet::readfloat() {
   uint32_t tmp = readu32();
   return *((float*)&tmp);
}

// utility functions

// do any final prep before transmitting a packet
void Packet::finish() {
   if( sz < buf_sz ) {
      buffer[sz++] = '\r';
   }
}

