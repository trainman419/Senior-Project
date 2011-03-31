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
#include "speedman.h"
#include "lock.h"

uint8_t brain_buffer[520];

// touch the internals of the serial library
extern uint8_t tx_lock[4];
void tx_internal(uint8_t port, uint8_t b);

/* Pass data from one uart to the other, without buffering it
   useful for datagrams that are too big to buffer, such as
   laser range data and GPS point lists
  
   This passthrough necessarily includes the terminating character,
   which is also passed

   In order to make sure all of our date gets passed properly, we reach into
   the serial library and manually acquire the transmit lock; passing in the 
   first byte allows us to transmit the entire packet without interruption
   */
void passthrough(uint8_t src, uint8_t dst, uint8_t first) {
   uint8_t input;
   // acquire lock on transmit device
   acquire_lock(tx_lock + dst);
   tx_internal(dst, first);
   while( (input = rx_byte(src)) != '\r' ) {
      tx_internal(dst, input);
   }
   tx_internal(dst, input);
   release_lock(tx_lock + dst);
}

// eat the data on a uart until a carriage return is found
void finish(int src) {
   // LED on
   PORTB |= (1 << 7);

   while( rx_byte(src) != '\r' );

   // LED off
   PORTB &= ~(1 << 7);
}

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
            // don't use passthrough, because laser data isn't properly escaped
            brain_buffer[0] = 'L';
            for( i=0; i<512; i++ ) {
               brain_buffer[i+1] = rx_byte(BRAIN);
            }
            brain_buffer[513] = '\r';
            tx_bytes(BT, brain_buffer, 514);
            while( rx_byte(BRAIN) != '\r' );
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
            while( rx_byte(BRAIN) != '\r' );
            break;
         case 'G':
            passthrough(BRAIN, BT, 'G');
            break;
      }
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
            finish(BT);
            break;
         case 'M':
            input = rx_byte(BT);
            {
               int8_t speed = (int8_t)input;
               speed = speed>100?100:speed;
               speed = speed<-100?-100:speed;

               //motor_speed(speed);
               target_speed = speed;
            }
            finish(BT);
            break;
         case 'S':
            input = rx_byte(BT);
            servo_set(0, input);
            finish(BT);
            break;
         case 'L':
            passthrough(BT, BRAIN, 'L');
            break;
      }

      /*do {
         while( !rx_ready(BT) ) yeild();
         input = rx_byte(BT);
      } while( input != '\r' && input != '\n' );
      */
   }
}
