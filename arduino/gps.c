/* gps.c
 * GPS library
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>
#include "serial.h"
#include "system.h"

uint8_t gps_port;

/* initialize GPS listener on serial port */
void gps_init(uint8_t port) {
   
   // initialize serial port and set baud rate
   serial_init(port);
   serial_baud(port, 2400);

   gps_port = port;

   // TODO: figure out which pins to use here
   // set /raw pin to output
   //DDRN |= 0x00;
   // set /raw pin high/low??
   //PORTN |= 0x00;
}

// buffer some characters of GPS data
#define GPS_BUFSZ 64
char gps_buffer[GPS_BUFSZ];

/* GPS listen thread */
void gps_thread(void) {
   uint8_t i=0;

   // main loop
   while(1) {
      while(rx_ready(gps_port)) {
         gps_buffer[i] = rx_byte(gps_port);

         // if we get a terminating character, process the data
         if(gps_buffer[i] == '\n') {
            // TODO: data processing here
            i = 0;
         }
         i++;

         // if we overrun our buffer, just limit to last index
         if( i >= GPS_BUFSZ ) i = GPS_BUFSZ-1;
      }
      yeild();
   }
}
