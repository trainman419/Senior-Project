/* protocol.cpp
 *
 * A cross-platform communication library for AVR and x86.
 *
 * Author: Austin Hendrix
 */

#include "protocol.h"

void Packet::append(unsigned char c) {
   if( c != '\r' ) {
      buffer[sz++] = c;
   } else {
      buffer[sz++] = esc;
      buffer[sz++] = c ^ esc;
   }
}

void Packet::append(signed char c) {
   append((unsigned char) c);
}

void Packet::append(unsigned short s) {
   unsigned char tmp;
   tmp = s & 0xFF;
   append(tmp);
   tmp = (s >> 8) & 0xFF;
   append(tmp);
}

void Packet::append(signed short s) {
   append((unsigned short) s);
}
