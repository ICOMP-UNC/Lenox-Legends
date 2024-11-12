/**
 * El Timer interrumpe cada cierto tiempo haciendo prender y apagar el led.
 */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

// Definiciones
#define TOGGLE_DELAY 1000

// Pines
#define LED_PORT GPIOC
#define LED_PIN  GPIO13

// Prototipos
void configure_gpio(void);
void configure_timer(void);
void tim2_isr(void);

/**
 * Manejo de desbordamiento de pila de FreeRTOS.
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask __attribute__((unused)),
                                   char *pcTaskName __attribute__((unused))) {
  while (1) {
  }
}

/**
 * Función principal
 */
int main(void) {
  rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Configuración del reloj

  configure_gpio();  // Configuración del GPIO para el LED
  configure_timer(); // Configuración del Timer 2

  while (1) {
  };
  return 0;
}

/**
 * @brief Configura el pin PC13 como salida para el LED.
 */
void configure_gpio(void) {
  rcc_periph_clock_enable(RCC_GPIOC); // Habilita el reloj para el puerto GPIOC
  gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
}

/**
 * @brief Configura el Timer 2 para generar interrupciones periódicas.
 */
void configure_timer(void) {
  rcc_periph_clock_enable(RCC_TIM2); // Habilita el reloj para el Timer 2

  timer_set_prescaler(TIM2, 7199); // Prescaler para un conteo de 10 kHz (72 MHz / 7200)
  timer_set_period(TIM2, 9999);    // Periodo para generar interrupciones cada 1 segundo (10 kHz / 10000)

  timer_enable_irq(TIM2, TIM_DIER_UIE); // Habilita la interrupción de actualización
  nvic_enable_irq(NVIC_TIM2_IRQ);       // Habilita la interrupción del Timer 2 en el NVIC

  timer_enable_counter(TIM2);           // Inicia el contador del Timer 2
}

/**
 * @brief Interrupción de Timer 2
 * Alterna el estado del LED en PC13 cada vez que ocurre una interrupción.
 */
void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_SR_UIF)) {   // Verifica si la interrupción fue generada por el flag de actualización
    timer_clear_flag(TIM2, TIM_SR_UIF);     // Limpia el flag de interrupción de actualización
    gpio_toggle(LED_PORT, LED_PIN);         // Alterna el estado del LED
  }
}

