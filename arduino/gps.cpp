/* gps.c
 * GPS library
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>

extern "C" {
#include "drivers/serial.h"
#include "main.h"
}

#include "ros.h"
#include <gps_simple/SimpleGPS.h>
#include "TinyGPS.h"

uint8_t gps_port;
gps_simple::SimpleGPS gps_msg;
ros::Publisher gps_pub("gps", &gps_msg);
char gps_frame[] = "";

/* initialize GPS listener on serial port */
void gps_init(uint8_t port) {
   // initialize serial port and set baud rate
   serial_init(port);
   serial_baud(port, 9600);

   gps_port = port;

   gps_msg.header.frame_id = gps_frame;
}

// output packet for GPS
//Packet<128> gps_packet('G');
TinyGPS gps;
uint8_t gps_input;
int32_t lat;
int32_t lon;

/* GPS listen thread */
void gps_spinOnce(void) {
   if(rx_ready(gps_port)) {
      gps_input = rx_byte(gps_port);

      if(gps.encode(gps_input)) {
         gps.get_position(&lat, &lon);

         gps_msg.latitude = lat;
         gps_msg.longitude = lon;
         // TODO: fill in rest of GPS message
         gps_pub.publish(&gps_msg);
      } 
   }
}
