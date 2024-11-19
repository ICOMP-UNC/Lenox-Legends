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
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdio.h>
#include "lcd_i2c.h"

//defines
#define RELOAD_COUNT 8999
#define TOGGLE_COUNT 500

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

void sys_tick_handler(void){
    systick_Count++;
    if(systick_Count > TOGGLE_COUNT){
        systick_Count = 0;
        gpio_toggle(GPIOC, GPIO13);
        print_lcd();
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

int main(void)
{
    systemInit();
    lcd_init();
    configure_pins();
    configure_systick();
    while (1)
    {
        
    }
}
