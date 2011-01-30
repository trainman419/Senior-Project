/* gps.h
 * GPS library
 *
 * Author: Austin Hendrix
 */

#ifndef GPS_H
#define GPS_H

/* initialize GPS listener on serial port */
void gps_init(uint8_t port);

/* GPS listener thread */
void gps_thread(void);

#endif
