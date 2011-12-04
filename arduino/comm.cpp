/* comm.c
 *
 * Communicaion threads
 *
 * Author: Austin Hendrix
 */

#include <stdint.h>

extern "C" {
#include "motor.h"
#include "drivers/servo.h"
#include "drivers/serial.h"
#include "drivers/power.h"
#include "system.h"
#include "comm.h"
#include "main.h"
#include "speedman.h"
#include "lock.h"
};

#include "protocol.h"

//uint8_t brain_buffer[520];
volatile int8_t steer; // steering setting
#define STEER_OFFSET 119

extern "C" {
   // touch the internals of the serial library
   extern uint8_t tx_lock[4];
   void tx_internal(uint8_t port, uint8_t b);

};

/* Pass data from one uart to the other, without buffering it
   useful for datagrams that are too big to buffer, such as
   laser range data and GPS point lists
  
   This passthrough necessarily includes the terminating character,
   which is also passed

   In order to make sure all of our date gets passed properly, we reach into
   the serial library and manually acquire the transmit lock; passing in the 
   first byte allows us to transmit the entire packet without interruption
   */
/*
void passthrough(uint8_t src, uint8_t dst, uint8_t first) {
   uint8_t input;

   // acquire lock on transmit device
   acquire_lock(tx_lock + dst);
   tx_internal(dst, first);
   while( (input = rx_byte(src)) != '\r' ) {
      tx_internal(dst, input);
      while(!rx_ready(src)) yeild();
   }
   tx_internal(dst, input);
   release_lock(tx_lock + dst);

}
*/

// eat the data on a uart until a carriage return is found
void finish(int src) {
   do {
      while(!rx_ready(src)) yeild();
   } while( rx_byte(src) != '\r' );
}

Packet<20> brain_pack(' ');

uint8_t bt_control = 1;

// Receive data from the brain
void brain_rx_thread(void) {
   uint8_t input;

   while(1) {
      // wait for input
      while(!rx_ready(BRAIN)) yeild();
      input = rx_byte(BRAIN);
      switch(input) {
         case 'M':
            // receive motor command
            brain_pack.reset();
            do {
               while(!rx_ready(BRAIN)) yeild();
               input = rx_byte(BRAIN);
               brain_pack.input((char)input);
            } while( input != '\r');
            if( !bt_control ) {
               target_speed = brain_pack.reads8();
               steer = brain_pack.readu8();
               input = steer + STEER_OFFSET; // steering zero set
               servo_set(0, input);
            }
            break;
         case 'G':
            passthrough(BRAIN, BT, 'G');
            break;
         default:
            passthrough(BRAIN, BT, input);
            break;
      }
   }
}

uint8_t buffer[1024];

// Receive data from bluetooth
void bt_rx_thread(void) {
   uint8_t input;
   uint16_t i;
   volatile uint16_t sz = 0;

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
               //tx_bytes(BRAIN, (uint8_t*)"ZZZZZZZZ\r", 9);
               while(sz != 0) yeild();
               sz = 9;
               brain_tx_buffer((uint8_t*)"ZZZZZZZZ\r", (uint16_t*)&sz);
               shutdown_count = 4*60;
            }
            finish(BT);
            break;
         case 'M':
            input = rx_byte(BT);
            if( bt_control ) {
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
            if( bt_control ) {
               servo_set(0, input);
               steer = input - STEER_OFFSET;
            }
            finish(BT);
            break;
         case 'C':
            bt_control = rx_byte(BT);
            finish(BT);
            break;
         case 'L':
            // at 8 bytes/pair, we can hold about 127 points
            buffer[0] = 'L';
            for( i=1, input=rx_byte(BT); 
                  i<1023 && input != '\r'; 
                  i++, input=rx_byte(BT) ) {
               buffer[i] = input;
            }
            buffer[i] = '\r';
            while( sz != 0 ) yeild();
            sz = i+1;
            brain_tx_buffer(buffer, (uint16_t*)&sz);
            //passthrough(BT, BRAIN, 'L');
            //finish(BT);
            break;
      }
   }
}
