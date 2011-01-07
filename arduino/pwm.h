/* Atmega2560 PWM Functions header

   Author: Austin Hendrix
*/

uint8_t pwm_init(uint8_t pin);
void pwm_off(uint8_t timer);
uint8_t pwm_set_duty(uint8_t pin, float duty);
uint8_t pwm_set_freq(uint8_t timer, uint16_t freq);

#define PWM2  0x31
#define PWM3  0x30
#define PWM4  0x01
#define PWM5  0x32
#define PWM6  0x42
#define PWM7  0x41
#define PWM8  0x40
#define PWM9  0x21
#define PWM10 0x22
#define PWM11 0x12
#define PWM12 0x11
#define PWM13 0x10

#define PWM14 0x50
#define PWM15 0x51
#define PWM16 0x52
