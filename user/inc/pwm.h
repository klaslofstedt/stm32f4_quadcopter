#ifndef PWM_h
#define PWM_h

void pwm_init(void);
void pwm_set_duty_cycle(uint8_t PinNum, uint16_t DutyCycle);
uint16_t pwm_get_prescaler(void);

#endif
