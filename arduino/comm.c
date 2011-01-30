/* comm.c
 *
 * Communicaion threads
 *
 * Author: Austin Hendrix
 */

#include <stdint.h>
#include "motor.h"
#include "servo.h"
#include "serial.h"
#include "power.h"
#include "system.h"
#include "comm.h"
#include "main.h"

uint8_t brain_buffer[520];

// Receive data from the brain
void brain_rx_thread(void) {
   uint8_t input;
   uint16_t i;

   while(1) {
      // apparently uncommenting this breaks things
      //while(!rx_ready(BRAIN)) yeild();
      input = rx_byte(BRAIN);
      switch(input) {
         case 'L':
            brain_buffer[0] = 'L';
            for( i=0; i<512; i++ ) {
               brain_buffer[i+1] = rx_byte(BRAIN);
            }
            brain_buffer[513] = '\r';
            tx_bytes(BT, brain_buffer, 514);
            break;
         case 'P':
            input = rx_byte(BRAIN);
            switch(input) {
               case '0':
                  break;
               case '1':
                  pwr_on();
                  break;
               case '2':
                  pwr_sleep();
                  break;
            }
            break;
      }
      while( rx_byte(BRAIN) != '\r' );
   }
}

// Receive data from bluetooth
void bt_rx_thread(void) {
   uint8_t input;
   uint16_t i;

   while(1) {
      while(!rx_ready(BT)) yeild();
      input = rx_byte(BT);
      switch(input) {
         case 'Z':
            input = 1;
            for( i=0; i<8; i++ ) {
               while(!rx_ready(BT)) yeild();
               if( rx_byte(BT) != 'Z' ) input = 0;
            }
            if( input ) {
               tx_bytes(BRAIN, (uint8_t*)"ZZZZZZZZ\r", 9);
               shutdown_count = 4*60;
            }
            break;
         case 'M':
            input = rx_byte(BT);
            {
               int8_t speed = (int8_t)input;
               speed = speed>50?50:speed;
               speed = speed<-50?-50:speed;

               motor_speed(speed);
            }
            break;
         case 'S':
            input = rx_byte(BT);
            servo_set(0, input);
            break;
      }

      do {
         while( !rx_ready(BT) ) yeild();
         input = rx_byte(BT);
      } while( input != '\r' && input != '\n' );

   }
}
