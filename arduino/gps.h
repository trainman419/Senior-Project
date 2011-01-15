/* gps.h
 * GPS library
 *
 * Author: Austin Hendrix
 */

/* initialize GPS listener on serial port */
void gps_init(uint8_t port);

/* GPS listener thread */
void gps_thread(void);
