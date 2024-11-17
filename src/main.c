#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdio.h>

#include "lcd_i2c.h"

//Creamos el taskHundle
TaskHandle_t led_handle = NULL;

#define tskLOW_PRIORITY ((UBaseType_t)tskIDLE_PRIORITY + 2)

// rutina para controlar el stack overflow
void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)), char *pcTaskName __attribute__((unused)))
{
    while (1)
    {
    }
}

void configure_pins()
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    // TODO: Agregar configuración de los pines
}

void configure_timer(void)
{
    rcc_periph_clock_enable(RCC_TIM2); // Habilita el reloj para el Timer 2

    timer_set_prescaler(TIM2, 7199); // Prescaler para un conteo de 10 kHz (72 MHz / 7200)
    timer_set_period(TIM2, 9999);    // Periodo para generar interrupciones cada 1 segundo (10 kHz / 10000)

    timer_enable_irq(TIM2, TIM_DIER_UIE); // Habilita la interrupción de actualización
    nvic_enable_irq(NVIC_TIM2_IRQ);       // Habilita la interrupción del Timer 2 en el NVIC
    nvic_set_priority(NVIC_TIM2_IRQ, 6);
    timer_enable_counter(TIM2); // Inicia el contador del Timer 2
    
}

static void task1(void *args __attribute__((unused)))
{
    while (1)
    {
        //Esperar la notificacion desde la ISR del timer
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)>0){
            //notificacion recibida
            gpio_toggle(GPIOC, GPIO13);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {                                       // Verifica si la interrupción fue generada por el flag de actualización
        timer_clear_flag(TIM2, TIM_SR_UIF); // Limpia el flag de interrupción de actualización

        BaseType_t xHigherPriorityTaskWoken = pdFALSE; //vble auxiliar. no se para que es
        //despertamos a la tarea
        vTaskNotifyGiveFromISR(led_handle, &xHigherPriorityTaskWoken);
        //Forzar cambio de contexto si una tarea de mayor prioridad fue despertada.
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

int main(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    configure_pins();
    configure_timer();
    // ENCENDEMOS el led
    gpio_set(GPIOC, GPIO13);

    // creamos las tareas
    xTaskCreate(task1, "LedSwitching", configMINIMAL_STACK_SIZE, NULL, tskLOW_PRIORITY, &led_handle);

    //iniciamos las tareas
    vTaskStartScheduler();

    while (1)
    {
    }
}
