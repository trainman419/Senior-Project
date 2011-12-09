#include <stdlib.h>
#include <avr/interrupt.h>
#include <drivers/serial.h>

int main() {
   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);

   sei();

   uint8_t * buffer;

   while(1) {
      // allocate a buffer, receive a byte, and echo it back
      buffer = (uint8_t*)malloc(1);
      buffer[0] = rx_byte(BRAIN);
      tx_buffer(BRAIN, buffer, 1);
   }
}
