/* sonar.c
 * Sonar Driver
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>
#include "serial.h"

#define NUMSONARS 5;
uint8_t sonar_port;
uint8_t sonar_value[NUMSONARS];

/* initalize sonar driver */
void sonar_init(uint8_t port) {
   serial_init(port);
   serial_baud(port, 9600);

   // sonar control pins
   DDRA = 0xFF;

   // set pins to all zeros
   PORTA = 0;
}

/* get the value of a sonar. return zero for non-existent sonars */
uint8_t get_sonar(uint8_t sonar) {
   if( sonar < NUMSONARS ) {
      return sonar_value[sonar];
   }
   return 0;
}
