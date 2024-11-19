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
#include <libopencm3/stm32/exti.h>
#include <stdint.h>
#include <stdio.h>
#include "lcd_i2c.h"

//defines
#define RELOAD_COUNT 8999
#define TOGGLE_COUNT 500

//magic numbers
#define BUFFER_SIZE 16

//exti
#define FALLING 0
#define RISING 1

static uint16_t exti_direction=FALLING;


//variables globales
volatile uint32_t systick_Count=0;
volatile uint16_t temperatura=0;
//Varibables para logica de puerta
volatile uint32_t contador_puerta=0;

enum EstadoPuerta{
    ABRIENDO,
    CERRANDO,
    DETENIDA
};

enum EstadoPuerta estado_puerta=DETENIDA;

bool modo=1; //0: manual, 1: automatico

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

void abrir_puerta();
void cerrar_puerta();
void parar_puerta();

bool a=false;

//implementaciones

void systemInit(){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void configure_pins()
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    //pines para la puerta
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
    gpio_clear(GPIOA, GPIO6);
    gpio_clear(GPIOA, GPIO7);

    //pines para el boton de modo
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO4);

    // TODO: Agregar configuración de los pines
}

void configure_systick(void){
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(RELOAD_COUNT);
    systick_interrupt_enable();
    systick_counter_enable();
}
void exti_setup(){
    rcc_periph_clock_enable(RCC_AFIO);
    nvic_enable_irq(NVIC_EXTI4_IRQ);
    exti_select_source(EXTI4, GPIO4);
    exti_set_trigger(EXTI4, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI4);
}
void exti4_isr(void){
    exti_reset_request(EXTI4);
    temperatura=0;
    a=!a;
    if(a){;
        estado_puerta=ABRIENDO;
    }
    else{
        estado_puerta=CERRANDO;
    }
}

void sys_tick_handler(void){
    systick_Count++;
    if(systick_Count > TOGGLE_COUNT){
        systick_Count = 0;
        gpio_toggle(GPIOC, GPIO13);
        print_lcd();
    }
    //Logica de puerta
    if(modo==1){
        if(estado_puerta==ABRIENDO){
            contador_puerta++;
            abrir_puerta();
            if(contador_puerta>TOGGLE_COUNT*6){
                estado_puerta=DETENIDA;
                parar_puerta();
                contador_puerta=0;
            }
        }
        else{
            if(estado_puerta==CERRANDO){
                contador_puerta++;
                cerrar_puerta();
                if(contador_puerta>TOGGLE_COUNT*6){
                    estado_puerta=DETENIDA;
                    parar_puerta();
                    contador_puerta=0;
                }
            }
        }
    }
}


void abrir_puerta(){
    gpio_clear(GPIOA, GPIO6);
    gpio_set(GPIOA, GPIO7);
}
void cerrar_puerta(){
    gpio_set(GPIOA, GPIO6);
    gpio_clear(GPIOA, GPIO7);
}
void parar_puerta(){
    gpio_clear(GPIOA, GPIO6);
    gpio_clear(GPIOA, GPIO7);
}
void print_lcd(){
    //preparamos las cosas para imprimir
    temperatura++;
    sprintf(buffer,"Temp: %d",temperatura);
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_print(buffer);

    //preparamos modo:
    sprintf(buffer,"Mode: %d",modo);
    lcd_set_cursor(1,0);
    lcd_print(buffer);
}

int main(void)
{
    systemInit();
    lcd_init();
    configure_pins();
    configure_systick();
    exti_setup();
    while (1)
    {

    }
}
