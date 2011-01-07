/* motor.h
 * Motor controller driver
 *
 * Low-level driver, deals with pwm, braking, direction control and error flags
 *
 * Author: Austin Hendrix
 */

#ifndef MOTOR_H
#define MOTOR_H

/* initialize the motor driver. */
void motor_init();

/* set the motor speed */
void motor_speed(int8_t speed);

/* get the status flags from the motor driver */
uint8_t motor_flags();

#endif
