#include <avr/io.h>

// E-stop is pin 53

void estop_init() {
   DDRB &= ~(1 << PB0); // PB0 as input
   PORTB |= (1 << PB0); // Pull-up PB0
}

int8_t estop() {
   return PINB & (1 << PB0);
}
