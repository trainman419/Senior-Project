/* sonar.c
 * Sonar Driver
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>
#include <util/delay.h>

extern "C" {
#include "drivers/serial.h"
}

#include "publish.h"

#define SONAR_TIMEOUT 200
#define SONAR_DELAY 20

#define NUM_SONARS 5
uint8_t sonar_port;
uint8_t sonar_value[NUM_SONARS];

uint8_t current_sonar;
uint8_t sonar_bytes;
uint8_t sonar_tmp;

extern uint32_t ticks;
uint32_t last_sonar = 0;

Publisher<12> sonar_pub('S');

/* initalize sonar driver */
void sonar_init(uint8_t port) {
   serial_init(port);
   serial_baud(port, 9600);

   // sonar control pins
   DDRA = 0xFF;

   // set pins to all zeros
   PORTA = 0;

   sonar_port = port;

   current_sonar = 0;
   sonar_bytes = 0; // number of bytes received
   sonar_tmp = 0;   // temporary sonar value

   PORTA = (0x80 >> current_sonar);
   _delay_us(SONAR_DELAY);
   PORTA = 0;
}

/* get the value of a sonar. return zero for non-existent sonars */
uint8_t get_sonar(uint8_t sonar) {
   if( sonar < NUM_SONARS ) {
      return sonar_value[sonar];
   }
   return 0;
}

void sonar_spinOnce(void) {
   // read the sonar buffer and deal with it
   while( rx_ready(sonar_port) ) {
      uint8_t input = rx_byte(sonar_port);
      switch(sonar_bytes) {
        case 0:
          break;
        case 1:
          sonar_tmp = (input - '0')*100;
          break;
        case 2:
          sonar_tmp += (input - '0')*10;
          break;
        case 3:
          sonar_tmp += (input - '0');
          break;
        case 4:
          if( input == '\r' ) {
            sonar_value[current_sonar] = sonar_tmp;
            // if this is the first sonar, reset
            if( current_sonar == 0 ) 
               sonar_pub.reset();

            sonar_pub.append(sonar_tmp);
            // if this is the last sonar, send
            if( current_sonar == (NUM_SONARS-1) )
               sonar_pub.finish();
          }
          break;
      }
      sonar_bytes++;

      // if we got a whole packet from our sonar, trigger the next one
      if( sonar_bytes > 4 ) {
        sonar_bytes = 0;
        current_sonar++;
        current_sonar %= NUM_SONARS;
        _delay_ms(2);
        PORTA = (0x80 >> current_sonar);
        _delay_us(SONAR_DELAY);
        PORTA = 0;
      }
   }
   if( ticks - last_sonar > SONAR_TIMEOUT ) {
      last_sonar = ticks;
      sonar_bytes = 0;
      PORTA = (0x80 >> current_sonar);
      _delay_us(SONAR_DELAY);
      PORTA = 0;
   }
}
