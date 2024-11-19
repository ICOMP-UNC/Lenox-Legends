/** @file main.c
 * 
 * @brief Programa principal del proyecto
 * 
*/
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <stdint.h>
#include <stdio.h>
#include "lcd_i2c.h"

//defines
#define RELOAD_COUNT 8999
#define TOGGLE_COUNT 500
#define TIMER_PRESCALER 7199 //prescaler para 10khz
#define TIMER_PERIOD 9999 //periodo para 1khz

//magic numbers
#define BUFFER_SIZE 16


//variables globales
volatile uint32_t systick_Count=0;
volatile uint16_t temperatura=0;

char buffer[BUFFER_SIZE];

//prototipos de funciones
/*
* @brief Inicializa el sistema
*
* @return void
*/
void systemInit(void);

/*
* @brief Configura los pines
*
* @return void
*/
void configure_pins(void);

/*
* @brief Configura el systick
*
* @return void
*/
void configure_systick(void);
/*
* @brief Handler del systick
*
* @return void
*/
void sys_tick_handler(void);
/*
* @brief Imprime en el LCD
*
* @return void
*/
void print_lcd(void);

//implementaciones

void systemInit(){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void configure_pins()
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    // TODO: Agregar configuración de los pines
}

void configure_systick(void){
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(RELOAD_COUNT);
    systick_interrupt_enable();
    systick_counter_enable();
}

void configure_timer(void) {
  rcc_periph_clock_enable(RCC_TIM2);       // Habilita el reloj para el Timer 2

  timer_set_prescaler(TIM2, TIMER_PRESCALER);         // Prescaler para un conteo de 10 kHz (72 MHz / 7200)
  timer_set_period(TIM2, TIMER_PERIOD);            // Periodo para generar interrupciones cada 1 segundo (10 kHz / 10000)

  timer_enable_irq(TIM2, TIM_DIER_UIE);    // Habilita la interrupción de actualización
  nvic_enable_irq(NVIC_TIM2_IRQ);          // Habilita la interrupción del Timer 2 en el NVIC

  timer_enable_counter(TIM2);              // Inicia el contador del Timer 2
}




int main(void)
{
    systemInit();
    lcd_init();
    configure_pins();
    configure_systick();
    configure_timer();
    while (1)
    {
        
    }
}

void print_lcd(){
    //preparamos las cosas para imprimir
    temperatura++;
    sprintf(buffer,"Temp: %d",temperatura);
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_print(buffer);

    //preparamos modo:
    sprintf(buffer,"Mode: %d",0);
    lcd_set_cursor(1,0);
    lcd_print(buffer);
}

void sys_tick_handler(void){
    systick_Count++;
    if(systick_Count > TOGGLE_COUNT){
        systick_Count = 0;
       // gpio_toggle(GPIOC, GPIO13);
        print_lcd();
    }
}

void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_SR_UIF)) {   // Verifica si la interrupción fue generada por el flag de actualización
    timer_clear_flag(TIM2, TIM_SR_UIF);     // Limpia el flag de interrupción de actualización

    //gpio_toggle(GPIOC, GPIO13);             // Cambia el estado del pin PC13

  }
}


