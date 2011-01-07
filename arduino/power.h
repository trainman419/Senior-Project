/* power.h
 *
 * Robot power management functions, including:
 *  5V main regulator enable/disable
 *  full power off
 *  battery monitoring
 *
 * Author: Austin Hendrix
 */

#ifndef POWER_H
#define POWER_H

/* enable 5V regulator */
void pwr_on();

/* disable 5V regulator */
void pwr_sleep();

/* full system shutdown. there is no going back! */
void pwr_off();

/* read charge of electronics battery. roughly 0-100 */
uint8_t main_battery();

/* read charge of motor battery. roughly 0-100 */
uint8_t motor_battery();

#endif
