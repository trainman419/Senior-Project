/* gps.c
 * GPS library
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>

extern "C" {
#include "serial.h"
#include "system.h"
#include "main.h"
}
#include "protocol.h"

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
Packet gps_packet('G');

/* GPS listen thread */
void gps_thread(void) {
   uint8_t input;

   // main loop
   while(1) {
      while(rx_ready(gps_port)) {
         input = rx_byte(gps_port);

         // if we get a terminating character, process the data
         if(input == '\n') {
            // finish and transmit packet
            gps_packet.finish();
            tx_bytes(BRAIN,
                  (const uint8_t *)gps_packet.outbuf(), 
                  gps_packet.outsz());

            // reset packet for next time
            gps_packet.reset();
         } else if( input != '\r' ) {
            gps_packet.append(input);
         }
      }
      yeild();
   }
}
