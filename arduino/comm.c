/* comm.c
 *
 * Communicaion threads
 *
 * Author: Austin Hendrix
 */

#include "comm.h"

uint8_t brain_buffer[520];

// Receive data from the brain
void brain_rx_thread(void) {
   uint8_t input;
   uint16_t i;

   while(1) {
      input = rx_byte(BRAIN);
      switch(input) {
         case 'L':
            brain_buffer[0] = 'L';
            for( i=0; i<512; i++ ) {
               brain_buffer[i+1] = rx_byte(BRAIN);
            }
            break;
         case 'P':
            break;
      }
      while( rx_byte(BRAIN) != '\r' );
   }
}

// Receive data from bluetooth
void bt_rx_thread(void) {


   while(1) {
   }
}
