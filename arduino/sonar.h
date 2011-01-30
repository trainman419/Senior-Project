/* sonar.h
 * Sonar Driver
 *
 * Author: Austin Hendrix
 */

#ifndef SONAR_H
#define SONAR_H

/* initalize sonar driver */
void sonar_init(uint8_t port);

// TODO: figure out if we want to read sonars on interrupts, or with a thread


/* get the value of a sonar */
uint8_t get_sonar(uint8_t sonar);

#endif
