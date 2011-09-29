/* gps.c
 * GPS library
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>

extern "C" {
#include "serial.h"
#include "main.h"
}
#include "protocol.h"
#include "TinyGPS.h"

uint8_t gps_port;

/* initialize GPS listener on serial port */
void gps_init(uint8_t port) {
   
   // initialize serial port and set baud rate
   serial_init_rx(port);
   serial_baud(port, 4800);

   gps_port = port;

   // set /raw pin to output
   DDRH |= (1 << 1);
   // set /raw pin low for raw mode
   PORTH &= ~(1 << 1);
   PORTH |= (1 << 0); // pull-up on input
}

// output packet for GPS
//Packet<128> gps_packet('G');
Packet<32> gps_packet('G');

/* GPS listen thread */
void gps_thread(void) {
   TinyGPS gps; // don't make this a global. not sure WHY :( FIXME
   uint8_t input;
   volatile uint16_t sz;
   int32_t lat;
   int32_t lon;

   // main loop
   while(1) {
      while(rx_ready(gps_port)) {
         input = rx_byte(gps_port);
         
         if(gps.encode(input)) {
            gps.get_position(&lat, &lon);

            gps_packet.reset();
            gps_packet.append(lat);
            gps_packet.append(lon);
            gps_packet.finish();

            sz = gps_packet.outsz();
            tx_buffer(BRAIN, (uint8_t*)gps_packet.outbuf(), sz);

//            while(sz != 0) yeild();
         } 

      }
//      yeild();
   }
}
