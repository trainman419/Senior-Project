/* Atmega2560 PWM Functions header

   Author: Austin Hendrix
*/

void pwm_init(unsigned char timer);
void pwm_off(unsigned char timer);
void pwm_set_duty(unsigned char timer, unsigned char duty);
void pwm_set_freq(unsigned char timer, unsigned int freq);
