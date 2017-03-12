#ifndef PWM_h
#define PWM_h

void pwm_init(void);
void pwm_input_init_tim2(void);
void pwm_input_init_tim5(void);
void pwm_input_init_tim12(void);
void pwm_input_init1(void);
void pwm_input_init2(void);
void pwm_input_init3(void);
void pwm_input_init4(void);

void pwm_set_duty_cycle(uint8_t PinNum, uint16_t DutyCycle);
uint16_t pwm_get_prescaler(void);

#endif
