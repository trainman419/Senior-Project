#define F_CPU 16000000UL

#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <drivers/serial.h>
#include <drivers/led.h>

int main() {
   led_init();
   led_off();

   serial_init(BRAIN);
   serial_baud(BRAIN, 115200);

   sei();

   uint8_t * buffer;

   while(1) {
      // allocate a buffer, receive a byte, and echo it back
      cli();
      buffer = (uint8_t*)malloc(1);
      sei();
      if( buffer ) {
        led_off();
        buffer[0] = rx_byte(BRAIN);
        tx_buffer(BRAIN, buffer, 1);
      } else {
        led_on();
      }
   }
   while(1);
}
