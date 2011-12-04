
#ifndef LED_H
#define LED_H

static inline void led_on() {
  PORTB |= (1 << PB7);
}

static inline void led_off() {
  PORTB &= ~(1 << PB7);
}
#endif
