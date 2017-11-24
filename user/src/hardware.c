#include "stm32f4xx.h"

#include "hardware.h"
#include "i2c.h"
#include "i2c_1.h"
#include "pwm.h"
#include "uart.h"
#include "debug.h"
#include "uart.h"
#include "freertos_time.h"
#include "buzzer.h"

// 168000000=168Mhz
extern uint32_t SystemCoreClock;

void hardware_init(void)
{
    // Setup STM32 system (clock, PLL and Flash configuration)
    SystemInit();
    // Update the system clock variable (might not have been set before)
    SystemCoreClockUpdate();
    
    //uart_init();

    //uart_printf("CoreClock: %d\n\r", SystemCoreClock);
    
    // VADFAN GÖR DENNA????????
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    
    // Allow access to Backup
    //PWR_BackupAccessCmd(ENABLE);
    
    // Reset RTC
    // RCC_BackupResetCmd(ENABLE);
    // RCC_BackupResetCmd(DISABLE);
    // Ensure all priority bits are assigned as preemption priority bits (FREERTOS)
    // This on is set to 4 according to FreeRTOS spec. Do not change.
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    //gpio_init();
    //debug_init();
    //pwm_output_init();
    //buzzer_init();

    //i2c2_init(); // imu
    //i2c1_init(); // barometer, lidar, laser

    //pwm_input_init(TIM2, GPIOA, GPIO_Pin_15);   // joystick CH2 = 32-bit pitch
    //pwm_input_init(TIM3, GPIOB, GPIO_Pin_4);    // joystick CH5 = 16-bit toggle
    //pwm_input_init(TIM5, GPIOA, GPIO_Pin_0);    // joystick CH1 = 32-bit roll    
    //pwm_input_init(TIM9, GPIOE, GPIO_Pin_5);    // joystick CH4 = 16-bit yaw
    //pwm_input_init(TIM12, GPIOB, GPIO_Pin_14);  // joystick CH3 = 16-bit thrust
    

    
}