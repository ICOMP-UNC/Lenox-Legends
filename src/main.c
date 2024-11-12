/**
 * El Timer 2 interrumpe cada 1 segundo para realizar la conversión por ADC en el pin A0.
 * Si el valor del ADC supera un umbral, el LED en PC13 se enciende o se apaga.
 */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>

// Definiciones
#define TOGGLE_DELAY 1000
#define ADC_THRESHOLD 2048  // Ajusta el umbral según la referencia (0-4095)

// Pines
#define LED_PORT GPIOC
#define LED_PIN  GPIO13

// Variables globales
uint16_t adc_value = 0;

// Prototipos
void configure_gpio(void);
void configure_timer(void);
void configure_adc(void);
uint16_t read_adc(void);
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
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]); // Configuración del reloj

  configure_gpio();  // Configuración del GPIO para el LED
  configure_adc();   // Configuración del ADC
  configure_timer(); // Configuración del Timer 2 para sincronizar el muestreo del ADC

  while (1) {
    // Aquí podrías realizar otras tareas mientras el ADC muestrea cada 1 segundo
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
 * @brief Configura el ADC en el canal 0 para leer el pin A0.
 */
void configure_adc(void) {
  rcc_periph_clock_enable(RCC_ADC1);       // Habilita el reloj para el ADC1

  adc_power_off(ADC1);                     // Apaga el ADC para configurarlo
  adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_1DOT5CYC); // Configura el tiempo de muestreo
  adc_set_regular_sequence(ADC1, 1, ADC_CHANNEL0); // Selecciona el canal 0 para el ADC

  adc_power_on(ADC1);                      // Enciende el ADC

  adc_reset_calibration(ADC1);             // Reinicia la calibración
  adc_calibrate(ADC1);                     // Calibra el ADC
}

/**
 * @brief Lee el valor actual del ADC en el canal 0.
 * @return Valor del ADC de 12 bits (0 a 4095).
 */
uint16_t read_adc(void) {
  adc_start_conversion_direct(ADC1);       // Inicia la conversión

  while (!adc_eoc(ADC1));                  // Espera a que termine la conversión

  return adc_read_regular(ADC1);           // Retorna el valor convertido
}

/**
 * @brief Configura el Timer 2 para generar interrupciones periódicas cada 1 segundo.
 */
void configure_timer(void) {
  rcc_periph_clock_enable(RCC_TIM2);       // Habilita el reloj para el Timer 2

  timer_set_prescaler(TIM2, 7199);         // Prescaler para un conteo de 10 kHz (72 MHz / 7200)
  timer_set_period(TIM2, 9999);            // Periodo para generar interrupciones cada 1 segundo (10 kHz / 10000)

  timer_enable_irq(TIM2, TIM_DIER_UIE);    // Habilita la interrupción de actualización
  nvic_enable_irq(NVIC_TIM2_IRQ);          // Habilita la interrupción del Timer 2 en el NVIC

  timer_enable_counter(TIM2);              // Inicia el contador del Timer 2
}

/**
 * @brief Interrupción de Timer 2.
 * Lee el valor del ADC cada segundo y controla el estado del LED en PC13
 * en función de si el valor del ADC supera el umbral.
 */
void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_SR_UIF)) {   // Verifica si la interrupción fue generada por el flag de actualización
    timer_clear_flag(TIM2, TIM_SR_UIF);     // Limpia el flag de interrupción de actualización

    adc_value = read_adc();                 // Lee el valor del ADC
    printf("ADC Value: %u\n", adc_value);   // Imprime el valor leído (opcional)

    // Control del LED en función del umbral del ADC
    if (adc_value > ADC_THRESHOLD) {
      gpio_clear(LED_PORT, LED_PIN);        // Apaga el LED si el valor ADC supera el umbral
    } else {
      gpio_set(LED_PORT, LED_PIN);          // Enciende el LED si el valor ADC está por debajo del umbral
    }
  }
}

