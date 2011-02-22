/* protocol.cpp
 *
 * A cross-platform communication library for AVR and x86.
 *
 * Author: Austin Hendrix
 */

#include <stdint.h>
#include "protocol.h"

// constructors
Packet::Packet(char * in, uint8_t in_sz) {
   uint8_t i;
   for( i=0; i<sz; i++) {
      buffer[i] = in[i];
   }
   sz = in_sz;
   idx = 1;
}

// append methods
void Packet::append(uint8_t c) {
   if( sz < 128 ) {
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

/*void Packet::append(volatile int16_t & s) {
   append((int16_t)s);
}

void Packet::append(volatile uint16_t & s) {
   append((uint16_t)s);
}*/

// read methods
unsigned char Packet::readu8() {
   unsigned char tmp = buffer[idx++];
   if( tmp != esc ) {
      return tmp;
   } else {
      tmp = buffer[idx++] ^ esc;
      return tmp;
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

// utility functions

// do any final prep before transmitting a packet
void Packet::finish() {
   buffer[sz++] = '\r';
}
