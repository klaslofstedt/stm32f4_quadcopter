#include "arm.h"
#include "buzzer.h"
#include <stdint.h>
#include "uart.h"
#include "stm32f4xx_it.h"
#include "freertos_time.h"

static uint8_t toggle = 0;
static uint16_t counter = 0;
static void arm_on_alert();
static void arm_off_alert();


static void arm_off_alert()
{
    uint8_t i;
    for(i = 0; i < 2; i++){
        uart_printf("arm off\n\r");
        buzzer_set(1000);
        delay_ms(400);
        buzzer_set(0);
        delay_ms(250);
    }
}

static void arm_on_alert()
{
    uint8_t i;
    for(i = 0; i < 3; i++){
        uart_printf("arm on\n\r");
        buzzer_set(1000);
        delay_ms(100);
        buzzer_set(0);
        delay_ms(100);
    }
}


uint8_t arm(void)
{
    // Read throttle
    float down = isr_read_duty(12);
    // Read yaw
    float right = isr_read_duty(9);
    // If both are biggest value (low & right on the controller), increase counter
    if(down > 11 && right > 11){
        counter++;
        if (counter > 500){ // 500*0.005 = 2.5sec
            if(toggle == 0){
                toggle = 1;
                arm_on_alert();
            }
            else{
                toggle = 0;
                arm_off_alert();
            }
            counter = 0;
        }
    }
    else{
        counter = 0;
    }
    
    if(toggle == 1){
        return 1;
    }
    return 0;
}