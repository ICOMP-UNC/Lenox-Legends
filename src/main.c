#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define tskLOW_PRIORITY ((UBaseType_t)tskIDLE_PRIORITY+2)

void vApplicationStackOverflowHook( TaskHandle_t pxTask __attribute__((unused)), char *pcTaskName __attribute__((unused))){
    while (1)
    {
    }
}

static void task1(void *args __attribute__((unused))){
    while(true){
        gpio_toggle(GPIOC, GPIO13);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    //ENCENDEMOS el led
    gpio_set(GPIOC, GPIO13);
    //configuramos rtos task
    xTaskCreate(task1,"LedSwitching",configMINIMAL_STACK_SIZE,NULL,tskLOW_PRIORITY,NULL);

    vTaskStartScheduler();

    while(1){

    };
    return 0;
}
