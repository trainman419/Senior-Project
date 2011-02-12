/* protocol.h
 *
 * A cross-platofrm AVR and x86 serial communication library.
 *
 * Author: Austin Hendrix
 */

class Packet {
   private:
      char buffer[64];
      unsigned char sz;

      const static char esc = 0x1B;

   public:
      // construct a packet, set first byte to type
      Packet(char type) { sz = 1; buffer[0] = type; }

      // construct a packet from a buffer
      Packet(char * in, unsigned char in_sz);

      // append various data types to a packet
      void append(unsigned char c);
      void append(signed char c);
      void append(unsigned short s);
      void append(signed short s);

      // read various types from a packet
      unsigned char  readu8();
      signed char    reads8();
      unsigned short readu16();
      signed short   reads16();

      // get data for sending packet
      const char * outbuf() { return buffer; }
      unsigned char outsz() { return sz; }
};
