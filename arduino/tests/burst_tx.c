#define F_CPU 16000000UL

#include <stdlib.h>
#include <drivers/serial.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main() {
   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);

   sei();

   int i;
   uint16_t tx_size = 10;
   uint8_t * tx_buf;
   uint8_t rx_buffer[1024]; // maximum measureable burst size

   _delay_ms(1500);

   while(1) {
      tx_buf = (uint8_t *)malloc(tx_size);

      for( i=0; i<tx_size; i++ ) tx_buf[i] = i;

      tx_buffer(BRAIN, tx_buf, tx_size);

      // delay to let things settle
      _delay_ms(250);

      rx_buffer[0] = rx_byte(BRAIN);
      rx_buffer[1] = rx_byte(BRAIN);
      tx_size = rx_buffer[0] | (rx_buffer[1] << 8);
      tx_size += 10;
   }
}
