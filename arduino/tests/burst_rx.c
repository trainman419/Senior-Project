#define F_CPU 16000000UL

#include <stdlib.h>
#include <drivers/serial.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main() {
   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);

   sei();

   uint16_t rx_size = 0;
   uint8_t * tx_buf;
   uint8_t rx_buffer[1024]; // maximum measureable burst size

   while(1) {
      // wait for incoming bytes
      while(!rx_ready(BRAIN)); 
      rx_size = 0;
      while(rx_ready(BRAIN)) {
         while(rx_ready(BRAIN)) {
            rx_buffer[rx_size] = rx_byte(BRAIN);
            rx_size++;
         }
         _delay_ms(1); // 1ms timeout for next byte
      }
      tx_buf = (uint8_t*)malloc(2);
      tx_buf[0] = rx_size & 0xff;
      tx_buf[1] = (rx_size >> 8) & 0xff;

      // send back number of received bytes
      tx_buffer(BRAIN, tx_buf, 2);
   }
}
