/* motor.c
 * Motor controller driver
 *
 * Low-level driver, deals with pwm, braking, direction control and error flags
 *
 * Author: Austin Hendrix
 */

#include <avr/io.h>
#include <math.h>

#include "pwm.h"
#include "motor.h"

/* initialize the motor driver. */
void motor_init() {
   // pwm pin as output
   DDRL |= (1 << 3) | (1 << 4) | (1 << 5);
   DDRL &= ~( (1 << 6) | (1 << 7) );

   PORTL |= (1 << 5);

   // motor pwm setup
   pwm_init(PWM16);
   pwm_set_freq(5, 10000); // 10kHz frequency
   pwm_set_duty(PWM16, 0.0);

}

/* set the motor speed */
void motor_speed(int8_t speed) {
   float duty;

   // limits on speed
   if( speed > 100 ) speed = 100;
   if( speed < -100 ) speed = -100;

   if( speed > 0 ) {
      duty = speed / 100.0;

      // set dir
      PORTL |= (1 << 4);
   } else {
      duty = -speed / 100.0;

      // clear dir
      PORTL &= ~(1 << 4);
   }
   pwm_set_duty(PWM16, duty);
}

/* get the status flags from the motor driver */
uint8_t motor_flags() {
   return PORTL >> 6;
}
