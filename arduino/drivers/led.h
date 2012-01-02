
#ifndef LED_H
#define LED_H

#include <avr/io.h>

static inline void led_init() {
   DDRB |= 1 << PB7;
}

static inline void led_on() {
  PORTB |= (1 << PB7);
}

static inline void led_off() {
  PORTB &= ~(1 << PB7);
}
#endif
