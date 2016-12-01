// C includes
#include "stdio.h"
// Hardware includes
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
// FreeRTOS kernel includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
// User includes
#include "i2c.h"
#include "pwm.h"
#include "uart.h"
#include "gpio.h"
#include "imu.h"
#include "motors.h"
#include "printf2.h"

static void init_hardware(void);
// 168000000=168Mhz
extern uint32_t SystemCoreClock; 


static void init_hardware(void)
{
    // Setup STM32 system (clock, PLL and Flash configuration)
    SystemInit();
    // Update the system clock variable (might not have been set before)
    SystemCoreClockUpdate();                               

    // Enable PWR APB1 Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    
    // Allow access to Backup
    PWR_BackupAccessCmd(ENABLE);
    
    // Reset RTC Do
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);
    
    //Configure Interrupts
    gpio_init();
    pwm_init();
    i2c_init(); 
    uart_init();
    printf2_init();
    
    // Ensure all priority bits are assigned as preemption priority bits
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
}

int main(void)
{
    // systick is 100000Hz = 100kHz = 0.1MHz
    init_hardware();
    
    //publish_queue = xQueueCreate(4, PUB_MSG_LEN);
    printf2("start\n\r");
    xTaskCreate(imu_task, (const char *)"imu_task", 1024, NULL, 2, NULL);
    xTaskCreate(motors_task, (const char *)"motors_task", 512, NULL, 2, NULL);

    vTaskStartScheduler();
    
    // should never be reached
    for(;;);
}



