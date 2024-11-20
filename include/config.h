#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>

#define RELOAD_COUNT 8999 // Reload Count para systick
#define TOGGLE_COUNT 500  // systick cada 500 ms
// Definiciones para el timer
#define TIMER_PRESCALER 7199  // prescaler para 10khz
#define TIMER_PERIOD_ADC 9999 // periodo para 1khz
#define BATERY_SENSE_TIME 3  // 1Leeremos la bateria cada 15 seg;
// Definiciones para el USART
#define USART_BAUDRATE 9600 // Baudrate 9600
#define USART_DATABITS 8    // 8 bits de datos
// exti
#define FALLING 0
#define RISING 1

#define MAX_COUNT 10000

//adc
#define ADC_MAX_VALUE 4095

void configure_hardware(void);
void configure_dma(uint16_t *adc_bateria);