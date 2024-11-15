#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <stdint.h>

#include "lcd_i2c.h"

//Definiciones de pins
//TO DO: Agregar definiciones de los pines

#define tskLOW_PRIORITY ((UBaseType_t)tskIDLE_PRIORITY+2)

//rutina para controlar el stack overflow
void vApplicationStackOverflowHook( TaskHandle_t pxTask __attribute__((unused)), char *pcTaskName __attribute__((unused))){
    while (1)
    {
    }
}

//asi se crean las tareas
static void task1(void *args __attribute__((unused))){
    while(true){
        gpio_toggle(GPIOC, GPIO13);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    rcc_periph_clock_enable(RCC_GPIOC);
  
    lcd_init();

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    //ENCENDEMOS el led
    gpio_set(GPIOC, GPIO13);


    //creamos las tareas
    xTaskCreate(task1,"LedSwitching",configMINIMAL_STACK_SIZE,NULL,tskLOW_PRIORITY,NULL);

    //iniciamos todas las tareas
    vTaskStartScheduler();
  
    volatile int contador=-1;
    char buffer[16];

    while(1){
        contador++;
        if(contador==5){
            lcd_clear();
            sprintf(buffer, "PRUE", contador);
        }
        else{
            lcd_clear();
            sprintf(buffer, "Contador: %d", contador);
        }
        lcd_set_cursor(0, 0);
        lcd_print(buffer);
        delay_ms(1000);
    }
}
