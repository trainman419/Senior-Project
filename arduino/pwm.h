/* Atmega2560 PWM Functions header

   Author: Austin Hendrix
*/

void pwm_init(unsigned char timer);
void pwm_off(unsigned char timer);
void pwm_set_duty(unsigned char timer, unsigned char duty);
void pwm_set_freq(unsigned char timer, unsigned int freq);

#define PWM2  0x3B
#define PWM3  0x3C
#define PWM4  0x0B
#define PWM5  0x3A
#define PWM6  0x4A
#define PWM7  0x4B
#define PWM8  0x4C
#define PWM9  0x2B
#define PWM10 0x2A
#define PWM11 0x1A
#define PWM12 0x1B
#define PWM13 0x0A

#define PWM14 0x5C
#define PWM15 0x5B
#define PWM16 0x5A
