/* servo.h
 *
 * Servo control routines, for producing servo PWM output on arbitrary pins
 *
 * Author: Austin Hendrix
 */

/* initialize the timer that generates servo interrupts */
void servo_init();

/* map a particular pin to a servo index */
void servo_map(uint8_t servo, volatile uint8_t * port, uint8_t pin);

/* set servo to the desired position */
void servo_set(uint8_t servo, uint8_t set);
